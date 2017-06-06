/**
 * @file lulu_kilobot.c
 * @brief Lulu P colony simulator extension that allows controlling a Kilobot robot by using a P colony based controller
 * In this file we implement all methods that allow a complete Kilobot application to be built.
 * @author Andrei G. Florea
 * @author Catalin Buiu
 * @date 2016-03-06
 */

#include "lulu_kilobot.h"
#include "instance.h"
#include "debug_print.h"

#ifdef SIMULATOR
    char* motionNames[] = {"stop", "straight", "left", "right"};
    char* colorNames[] = {"off", "red", "green", "blue", "white"};
#endif

//get *mydata to the coresponding USERDATA structure of the running robot
REGISTER_USERDATA(USERDATA)

void set_motion(motion_t dir_new)
{
	//if the same direction is requested again
	if (dir_new == mydata->current_motion_state)
		//skip request in order to avoid an unneeded engine spinup
		return;
	switch (dir_new)
	{
		case MOTION_STOP:
			set_motors(0, 0);
			break;

		case MOTION_STRAIGHT:
			spinup_motors();
			set_motors(kilo_straight_left, kilo_straight_right);
			break;

		case MOTION_LEFT:
			spinup_motors();
			set_motors(kilo_turn_left, 0);
			break;

		case MOTION_RIGHT:
			spinup_motors();
			set_motors(0, kilo_turn_right);
			break;
	}
	mydata->current_motion_state = dir_new;
}

void handle_default() {
    printd("called default handler");
}

void handle_neighbor_close() {
    uint8_t rand_value, options_count = 4;
    printd("handle neighbor close");
    //set_color(RGB(3, 0, 0));
    //mydata->current_led_color = COLOR_RED;

#ifndef KILOBOT
    //use rand() from stdlib.h
    //rand_value in [0; options_count-1] interval
    rand_value = rand() % options_count;
#else
    //use rand_soft from kilolib.h
    //rand_value in [0; options_count-1] interval
    rand_value = rand_soft() % options_count;
#endif

    switch (rand_value) {
        case 0: case 1: {
            mydata->current_led_color = COLOR_GREEN;
            set_motion(MOTION_STRAIGHT);
            break;
        }
        case 2: {
            mydata->current_led_color = COLOR_RED;
            set_motion(MOTION_LEFT);
            break;
        }
        case 3: {
            mydata->current_led_color = COLOR_BLUE;
            set_motion(MOTION_RIGHT);
            break;
        }
    }

    set_color(colorValues[mydata->current_led_color]);
}

void handle_all_neighbors_distant() {
    printd("handle neighbor distant");
    set_color(RGB(3, 3, 3));
    mydata->current_led_color = COLOR_WHITE;
    set_motion(MOTION_STOP);
}

void forget_neighbors() {
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        // if the deadline for forgeting this neighbor has passed
        if (kilo_ticks >= mydata->neighbors[i].timexp_forget && mydata->neighbors[i].timexp_forget > 0) {
            //decrease the number of neighbors
            mydata->nr_neighbors = (mydata->nr_neighbors > 0)? mydata->nr_neighbors - 1: 0;
            //re-initialize this neighbor
            mydata->neighbors[i].uid = NO_ID;
            mydata->neighbors[i].symbolic_id = NO_ID;
            mydata->neighbors[i].distance = 0;
            mydata->neighbors[i].distance_prev = 0;
            mydata->neighbors[i].timexp_forget = 0;
        }
}

void process_message() {
    uint8_t i;


    uint8_t distance = estimate_distance(&RB_front().dist);
    uint8_t *data = RB_front().msg.data;
    uint16_t id = data[INDEX_MSG_OWNER_UID_LOW] | data[INDEX_MSG_OWNER_UID_HIGH] << 8;

    //search for the robot uid in the current neighbor list
    for (i = 0; i < MAX_NEIGHBORS; i++)
        if (mydata->neighbors[i].uid == id)
            break;

    //if the sender of this message was not found in the current list of neighbors
    if (i == MAX_NEIGHBORS)
        //check for an empty slot
        for (i = 0; i < MAX_NEIGHBORS; i++)
            if (mydata->neighbors[i].uid == NO_ID) {
                //we will remember a new neighbor
                mydata->nr_neighbors++;
                break;
            }

    //if there was no empty slot
    if (i == MAX_NEIGHBORS) {
        printw("kilo_uid: %d - no slot for KB%d", kilo_uid, id);
        return;
    }

    //if we reach this step then i is a valid slot
    mydata->neighbors[i].uid = id;
    //mydata->neighbors[i].symbolic_id = id - smallest_robot_uid;
    mydata->neighbors[i].symbolic_id = id;
    //if there is no previous recording of the distance to this neighbor
    if (mydata->neighbors[i].distance == mydata->neighbors[i].distance_prev &&
           mydata->neighbors[i].distance == 0)
        //save current distance in both fields
        mydata->neighbors[i].distance = mydata->neighbors[i].distance_prev = distance;
    else {
        //save previous distance
        mydata->neighbors[i].distance_prev = mydata->neighbors[i].distance;
        //save current distance
        mydata->neighbors[i].distance = distance;
    }

    //set the moment in the future when the robot will forget about this neighbor
    mydata->neighbors[i].timexp_forget = kilo_ticks + FORGET_NEIGHBOR_INTERVAL;
}

void procInputModule() {
    uint8_t i; //used for iterating through the neighbor list
    bool dist_big = TRUE;

    for (i = 0; i < MAX_NEIGHBORS; i++)
        if (mydata->neighbors[i].distance < PARAM_DISTANCE_THRESHOLD && mydata->neighbors[i].uid != NO_ID) {
            dist_big = FALSE;
            break;
        }
    //if (dist_big || mydata->nr_neighbors == 0)
    if (dist_big)
        //replaceObjInMultisetObj(&mydata->pcol.agents[AGENT_MSG_DISTANCE].obj, OBJECT_ID_D_ALL, OBJECT_ID_B_ALL);
        mydata->current_event = EVENT_ALL_NEIGHBORS_DISTANT;
    else
        //replaceObjInMultisetObj(&mydata->pcol.agents[AGENT_MSG_DISTANCE].obj, OBJECT_ID_D_ALL, OBJECT_ID_S_ALL);
        mydata->current_event = EVENT_NEIGHBOR_CLOSE;
}

message_t* message_tx() {
    return &mydata->msg_tx;
}

void message_rx(message_t* msg, distance_measurement_t* dist) {
    //add one message at the tail of the receive ring buffer
    Received_message_t *newMsg = &RB_back();
    newMsg->msg = *msg;
    newMsg->dist = *dist;
    //now RB_back() will point to an empty slot in the buffer
    RB_pushback();
}

void setup_message() {
    mydata->msg_tx.type = NORMAL;
    mydata->msg_tx.data[INDEX_MSG_OWNER_UID_LOW] = kilo_uid & 0xFF; //low byte
    mydata->msg_tx.data[INDEX_MSG_OWNER_UID_HIGH] = kilo_uid >> 8; //high byte
    mydata->msg_tx.crc = message_crc(&mydata->msg_tx);
}

void loop() {
#ifdef PCOL_SIM
    printi("\nLOOP for robot %d\n-------------------------\n", kilo_uid);
#endif
    //if the previous step was the last one
    //if (mydata->sim_result == SIM_STEP_RESULT_NO_MORE_EXECUTABLES) {
        ////mark the end of the simulation and exit
        //set_color(RGB(0, 2, 0));
        //set_motion(MOTION_STOP);
        //printi("kilo_uid %d: NO_MORE_EXEC", kilo_uid);
        //return;
    //} else
    //// if the previous step resulted in an error
    //if (mydata->sim_result == SIM_STEP_RESULT_ERROR) {
        //set_color(RGB(2, 0, 0));
        //set_motion(MOTION_STOP);
        ////we don't print any error message here because we wouldn't be able to read the LULU error because this continuous print
        //return;
    //}

    //remove old neighbors that haven't sent any recent message
    forget_neighbors();

    //process the entire received message buffer
    while (!RB_empty()) {
        process_message();
        RB_popfront();
    }

#ifdef USING_ID_SECURITY
    if (kilo_uid == STRANGER_UID) {
        set_color(RGB(3, 0, 3)); //magenta
        return;
    }
#endif
    //transform sensor input into symbolic objects
    procInputModule();

    handlers[mydata->current_event]();
#ifdef KILOBOT
    //prevent the real kilobot from moving chaotically because of a too high sim step execution speed
    delay(SLEEP_MS_BETWEEN_SIMSTEPS);
#endif
}

void setup() {
    //initialize the mydata structure
    //mydata->current_led_color = COLOR_OFF;
    //mydata->current_motion_state = MOTION_STOP;
    //mydata->sim_result = SIM_STEP_RESULT_FINISHED;
    //mydata->light = mydata->light_prev = 0;
    mydata->nr_neighbors = 0;
    mydata->neighbor_index = 0;
    mydata->current_event = EVENT_ALL_NEIGHBORS_DISTANT;
    //initialize message for transmission
    setup_message();


    //init neighbors
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        mydata->neighbors[i] = (Neighbor_t) {
            .uid = NO_ID,
            .symbolic_id = NO_ID,
            .distance = 0,
            .distance_prev = 0,
            .timexp_forget = 0};

    //initialize message receive buffer
    RB_init();
}

#ifdef SIMULATOR
/* provide a text string for the simulator status bar about this bot */
char *cb_botinfo(void)
{
    char *p = botinfo_buffer;
    p += sprintf (p, "ID: %d, MOTION: %s, COLOR: %s \n", kilo_uid, motionNames[mydata->current_motion_state], colorNames[mydata->current_led_color]);

    p += sprintf (p, "\n nr_neighbors = %d ", mydata->nr_neighbors);
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++)
        if (mydata->neighbors[i].uid != NO_ID)
            p += sprintf (p, "n[%d]={%d, %d}, ", i, mydata->neighbors[i].uid, mydata->neighbors[i].distance);
    return botinfo_buffer;
}
#endif

int main() {
    kilo_init();
    #ifdef DEBUG
        // initialize serial module (only on Kilobot)
	    debug_init();
    #endif
    //callbacks for Kilombo (resolve to empty functions if building for Kilobot)
    SET_CALLBACK(botinfo, cb_botinfo);

    //register kilobot callbacks
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;

    kilo_start(setup, loop);

    return 0;
}
