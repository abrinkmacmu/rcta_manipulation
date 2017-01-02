#ifndef __state__machine__
#define __state__machine__

namespace smach{

enum SmachPrintEvent {
	States,
	All,
}

enum Transitions {
	Failure,
	Success,
	Continue,
	Complete,
}

const void smachPrint(SmachPrintEvent, std::string msg);

class State{
public:

	virtual executeState();

private:
	std::string state_name_;

}; // Class State

class UserData{};



class StateMachine{
public:
	void registerState(std::string state_name, *State state);
	void registerTransition(Transitions transition, std::string state);
	void runStartingFromState(std::string start_state);

private:
	std::map<std::string, *State> state_map_; //< client is responsible for the state instances
	std::vector<std::string> state_names_;


}; // Class StateMachine


// instance of stateMachine at smach::stateMachine
StateMachine stateMachine;

} // ns smach

#endif