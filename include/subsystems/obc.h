/**
 * @file obc.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef INC_OBC_H_
#define INC_OBC_H_

// TODO: revisar stack sizes y prioridades!

// Task stack sizes
#define OBC_STACK_SIZE      1024  
#define PAYLOAD_STACK_SIZE  1024  
#define EPS_STACK_SIZE      1024
#define COMMS_STACK_SIZE    1024   
#define OBDH_STACK_SIZE     1024   


// Task priorities
#define OBC_PRIORITY        2
#define PAYLOAD_PRIORITY    1
#define EPS_PRIORITY        1
#define COMMS_PRIORITY      1
#define OBDH_PRIORITY       1

/**
 * @brief Communications task function, it runs the OBC state machine.
 */
void obc_task(void *pv_parameters);

#endif /* INC_OBC_H_ */