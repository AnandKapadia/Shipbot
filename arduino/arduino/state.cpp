/** @file state.cpp
 *  @brief The library to handle system state and actions
 *  associated with each state
 *
 *  @author Anand Kapadia
 *  @author ...add names here
 */
 
 #include <"vars.h">

 /**
 * @brief Variable that defines what functional state the robot is in
 * ... use the type systemState to define this variable.
 */
systemState state;

void update_system_state(systemState s){
	//update the system state
	state = s;

	//based on new system state, take the following action...
	switch (s){
		case INIT:
			break;
		case LOCALIZE:
			break;
		case PATH_PLAN:
			break;
		case ROBOT_MOTION:
			break;
		case ARM_MOTION:
			break;
		case DONE:
			break;
		default:
			break;
	}
}

systemState get_system_state(){
	return state;
}