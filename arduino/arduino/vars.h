/** @file vars.h
 *  @brief All types and definitions for the system
 *
 *  @author Anand Kapadia
 */

#ifndef _vars_h_
#define __vars_h_

#include <stdio.h>

/**
 * @brief State to define the current robot system function/task
 */
typedef enum {
	INIT,
	LOCALIZE,
	PATH_PLAN,
	ROBOT_MOTION,
	ARM_MOTION,
	DONE
} systemState;


#endif
