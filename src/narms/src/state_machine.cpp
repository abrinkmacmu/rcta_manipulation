#include <iostream>
#include <iomanip>
#include <ctime>
#include "state_machine.h"
#include <sstream>

std::string smach::getCurrentDateString(){
	// get time and date
	std::time_t now = time(0);
	std::tm *ltm = localtime(&now);
	std::string date_str;
	date_str.append( std::to_string(1900 + ltm->tm_year));
	date_str.append("_");
	date_str.append(std::to_string(1 + ltm->tm_mon));
	date_str.append("_");
	date_str.append(std::to_string(ltm->tm_mday));
	date_str.append("_");
	date_str.append(std::to_string(ltm->tm_hour));
	date_str.append(":");
	date_str.append(std::to_string(ltm->tm_min));
	date_str.append(":");
	date_str.append(std::to_string(ltm->tm_sec));
	return date_str;
}


smach::StateMachine::StateMachine()
{
	std::cout << "stateMachine started up\n";

	std::string time_str = getCurrentDateString();

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
	};

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
		std::cerr << "Specified state " << start_state << " not found, register state first!\n";
	}

	std::string currentState(start_state);
	for(;;){
		std::cout << currentState << "\n";
		Transition T = state_map_[currentState]->executeState();
		currentState = transition_map_[currentState][T];
		if(0 == currentState.compare(INVALID_TRANSITION))
		{
			std::cerr << "State transition is invalid!\n";
			return;
		}
		if(0 == currentState.compare(COMPLETED_TRANSITION))
		{
			std::cout << "State machine completed successfully\n";
			return;
		}
	}
}

bool smach::StateMachine::isStateFound(std::string state)
{
	bool state_found = false;
	for(auto it = state_names_.begin();  it != state_names_.end(); it++)
	{
		if(0 == (*it).compare(state)){ state_found = true;}
	}
	return state_found;
}
