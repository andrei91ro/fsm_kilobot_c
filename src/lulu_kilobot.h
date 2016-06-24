// vim:filetype=c
/**
 * @file lulu_kilobot.h
 * @brief Lulu P colony simulator extension that allows controlling a Kilobot robot by using a P colony based controller
 * In this header we declare the structure and functions of a basic Kilobot controller that is based on P colonies and implements a bidirectional symbolic object - numeric value conversion.
 * This application uses the Kilombo simulator and also the kilolib library for interfacing with both a simulated and real kilobot.
 * @author Andrei G. Florea
 * @author Catalin Buiu
 * @date 2016-03-06
 */

#ifndef LULU_KILOBOT_H
#define LULU_KILOBOT_H

//on the Kilobot, debug functions such as printf have to be specifically included with debug.h
#if defined(DEBUG_PRINT) && defined(KILOBOT)
    //enables UART serial module
    #define DEBUG
    #include "debug.h"
#endif

#include <kilombo.h> //for access to sensors, motors (includes kilolib.h)
#include "lulu.h"

#define MAX_NEIGHBORS 20
#define RB_SIZE 8 //ring buffer size

//we define a NO_ID value for Neighbor_t.uid, because 0 can be used as a regular uid
#define NO_ID UINT16_MAX

#ifdef USING_ID_SECURITY
    //the stranger robot doesn't move (is moved by the user), emits messages as normal robots and is magenta coloured
    //this robot is used in ID based security demos
    #define STRANGER_UID 2
    //#define STRANGER_UID 62
#endif

#define FORGET_NEIGHBOR_INTERVAL 32 * 2 //forget neighbors if the last msg received was 32 * X seconds ago (1 second = 32 kiloticks)
#define SLEEP_MS_BETWEEN_SIMSTEPS 50 //the number of miliseconds to wait before executing the next Lulu simulation step

/**
 * @brief Define the states used for a robot
 */
typedef enum {
    STATE_STOP_LED_OFF,
    STATE_LEFT_RED,
    STATE_RIGHT_BLUE,
    STATE_STRAIGHT_GREEN,
    STATE_STOP_WHITE
} state_t;

/**
 * @brief Define the events that can be perceived
 */
typedef enum {
    EVENT_NEIGHBOR_CLOSE,
    EVENT_ALL_NEIGHBORS_DISTANT
} event_t;

/**
 * @brief Default finite state machine event handler - does not change anything
 */
void handle_default();

void handle_neighbor_close();

void handle_all_neighbors_distant();

void (*handlers[2])(void) = {handle_neighbor_close, handle_all_neighbors_distant};


/**
 * @brief Define motion types
 */
typedef enum {
    MOTION_STOP,
    MOTION_STRAIGHT,
    MOTION_LEFT,
    MOTION_RIGHT
} motion_t;

typedef enum {
    COLOR_OFF,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_WHITE
    //TODO add all basic colors used in lulu_kilobot.py
} led_color_t;

uint8_t colorValues[] = {RGB(0, 0, 0), RGB(3, 0, 0), RGB(0, 3, 0), RGB(0, 0, 3), RGB(3, 3, 3)};

enum {
    INDEX_MSG_OWNER_UID_LOW,
    INDEX_MSG_OWNER_UID_HIGH,
};

/**
 * @brief Structure that holds information about a recent neighbor robot
 */
typedef struct _Neighbor {
    uint16_t uid;
    uint16_t symbolic_id;
    uint8_t distance, distance_prev;
    uint32_t timexp_forget;
} Neighbor_t;

/**
 * @brief Structure that holds information about a received message
 * This information cannot be processed imediately after receiving because estimating the ditance is a time consuming task that cannot be executed inside an intrerupt.
 */
typedef struct _received_message {
    message_t msg;
    distance_measurement_t dist;
} Received_message_t;

typedef struct {
    Pcolony_t pcol;

    //uint8_t light,
            //light_prev;
    motion_t current_motion_state;
    led_color_t current_led_color;
    //sim_step_result_t sim_result;

    Neighbor_t neighbors[MAX_NEIGHBORS];
    uint8_t nr_neighbors,
            neighbor_index;

    //transmitted message
    message_t msg_tx;

    //receive messages ring buffer (taken from kilombo/examples/gradient2)
    uint8_t RXHead, RXTail;
    Received_message_t RXBuffer[RB_SIZE];

    //the currently perceived event
    event_t current_event;

} USERDATA;

/*-------------------- RING BUFFER DEFINITION ---------------------------*/
//this definition was taken from kilombo/examples/gradient2/gradient.h
//available at: https://github.com/JIC-CSB/kilombo/blob/master/examples/gradient2/gradient.h

// Ring buffer operations. Taken from kilolib's ringbuffer.h
// but adapted for use with mydata->

// Ring buffer operations indexed with head, tail
// These waste one entry in the buffer, but are interrupt safe:
//   * head is changed only in popfront
//   * tail is changed only in pushback
//   * RB_popfront() is to be called AFTER the data in RB_front() has been used
//   * head and tail indices are uint8_t, which can be updated atomically
//     - still, the updates need to be atomic, especially in RB_popfront()

#define RB_init() {	\
    mydata->RXHead = 0; \
    mydata->RXTail = 0;\
}

#define RB_empty() (mydata->RXHead == mydata->RXTail)

#define RB_full()  ((mydata->RXHead+1)%RB_SIZE == mydata->RXTail)

#define RB_front() mydata->RXBuffer[mydata->RXHead]

#define RB_back() mydata->RXBuffer[mydata->RXTail]

#define RB_popfront() mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;


#define RB_pushback() {\
    mydata->RXTail = (mydata->RXTail+1)%RB_SIZE;\
    if (RB_empty())\
      { mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;	\
	/*printf("Full.\n"); */}			\
  }

/*-------------------- END RING BUFFER DEFINITION -----------------------*/

#ifdef SIMULATOR
    /* provide a text string for the simulator status bar about this bot */
    static char botinfo_buffer[10000];
    extern char* motionNames[];
    extern char* colorNames[];
#endif

//TODO params need tunning
#define PARAM_LIGHT_THRESHOLD 20
#define PARAM_DISTANCE_THRESHOLD 55

//end event handlers

/**
 * @brief Remove old neighbors from the neighbor array.
 * Old neighbors are considered neighbors from which a message was received more than FORGET_NEIGHBOR_INTERVAL kilotics ago.
 *
 */
void forget_neighbors();

/**
 * @brief Process received data and transfer it to the Neighbor_t data structures for further processing and symbolic conversion
 *
 */
void process_message();

/**
 * @brief Process raw_state info received from sensors and populate the input module agents with significant objects
 *
 */
void procInputModule();

/**
 * @brief Process the objects present in the output module agents and transform them into commands that are executed by the Kilobot
 *
 */
void procOutputModule();


/**
 * @brief Set the motion direction
 *
 * @param dir_new The new motion direction (DIR_{STOP, FORWARD, LEFT, RIGHT})
 */
void set_motion(motion_t dir_new);

/**
 * @brief Function called after receiving a new message
 *
 * @param msg Pointer to the new message
 * @param dist Pointer to the structure that allows to determine the distance from the sender
 */
void message_rx(message_t* msg, distance_measurement_t* dist);

/**
 * @brief Function called when preparing a new message
 *
 * @return Pointer to the new message
 */
message_t* message_tx();

/**
 * @brief Function called after the message has been succesfully sent
 */
void message_tx_success();

/**
 * @brief Prepares a new message for sending (with CRC)
 */
void setup_message();

/**
 * @brief Function called in a continuos loop during the runtine of the program
 */
void loop();

/**
 * @brief Function called only once at startup, before entering the continuos loop
 */
void setup();

void process_message();

#ifdef SIMULATOR
/**
 * @brief Prepares a status bar message for the simulator
 *
 * @return The status bar message
 */
char *cb_botinfo(void);
#endif

#endif
