#include <iostream>
#include <iomanip>
#include <ctime>
#include <state_machine.h>
#include <sstream>

smach::StateMachine::StateMachine()
{
	std::cout << "stateMachine started up\n";

	auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << put_time(&tm, "%d-%m-%Y %H-%M-%S");
    std::string time_str(oss.str() );

    text_file_ = "text_" + time_str;
    dot_file_ = "dot_" + time_str;

}

void smach::StateMachine::registerState(std::string state_name, State* state)
{
	// check if state is already in list of state_names
	if (isStateFound(state_name))
	{
		std::cerr << "Specified state " << state << " already registered!\n";
	}

	state_names_.push_back(state_name);
	state_map_[state_name] = state;

	std::map<Transition, std::string> trans {
		{Failure,  INVALID_TRANSITION},
		{Success,  INVALID_TRANSITION},
		{Continue, INVALID_TRANSITION},
		{Complete, INVALID_TRANSITION},
	}

	transition_map_[state_name] = trans;
}

void smach::StateMachine::registerTransition(
	std::string state, Transition transition, std::string dest)
{

	// check if state is not in list of state_names
	
	if (!isStateFound(state))
	{
		std::cerr << "Specified state " << state << " not found, register state first!\n";
	}


	transition_map_[state][transition] = dest;
}

void smach::StateMachine::runStartingFromState(std::string start_state)
{

	if(!isStateFound(start_state))
	{
		std::cerr << "Specified state " << state << " not found, register state first!\n";
	}

	std::string currentState(start_state);
	for(;;){
		Transition T = state_map_[currentState]->executeState();
		currentState = transition_map_[currentState][T];
		if(currentState == INVALID_TRANSITION)
		{
			std::cerr << "State transition is invalid!\n";
			return;
		}
		if(currentState == COMPLETED_TRANSITION)
		{
			std::cout << "State machine completed successfully\n";
			return;
		}
	}
}

bool isStateFound(std::string state)
{
	bool state_found = false;
	for(auto it = state_names_.begin();  it != state_names_.end(); it++)
	{
		if(it == state){ state_found = true;}
	}
	return state_found;
}
