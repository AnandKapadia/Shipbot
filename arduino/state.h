/** @file state.h
 *  @brief State Function prototypes
 *
 *  @author Anand Kapadia
 */

#ifndef _state_h_
#define __state_h_

#include <stdio.h>
#include "vars.h"

/**
 * @brief      Update the current robot state
 *
 * @param[in]  s     State which the robot should enter
 */
void update_system_state(systemState s);

/**
 * @brief      Returns the current system state
 *
 * @return     The system state.
 */
systemState get_system_state();

#endif