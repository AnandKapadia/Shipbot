/** @file arduino.ino
 *  @brief The main program for the arduino mega
 *
 *  @author Anand Kapadia
 *  @author ...add names here
 */

#include "vars.h"
#include "state.h"
#include "serial.h"
#include "localize.h"
#include <"pathplan.h"> 

/**
 * @brief      Setup and Initialize all pins, firmware, etc.
 */
void setup() {
  //initialize state
  update_system_state(INIT);

  //initialize all subsystems
  initialize_pi_serial();


}

void loop() {
	
}
