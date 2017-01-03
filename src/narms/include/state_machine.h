#ifndef __state__machine__
#define __state__machine__

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ctime>


namespace smach {

std::string getCurrentDateString();


enum Transition {
	Failure,
	Success,
	Continue,
	Complete
};

const std::string INVALID_TRANSITION("INVALID/NOTSET");
const std::string COMPLETED_TRANSITION("COMPLETED");


/**
 *  @brief State is meant to be publicly inherited by any implemented state, should only have to override pure virtual executeState method
 */
class State {
public:
	State() {};

	virtual Transition executeState() = 0;
	virtual ~State() {};

private:

}; // Class State





/**
 * @brief StateMachine class manages transitions and states, logs progress, and prints debug messages
 */
class StateMachine {
public:
	StateMachine();
	void setLogFilePath(std::string fn) {log_path_ = fn; };
	void registerState(std::string state_name, State* state);
	void registerTransition(std::string state, Transition transition, std::string dest);
	void runStartingFromState(std::string start_state);

private:
	std::map<std::string, State*> state_map_; //< client is responsible for the state instances, TODO consider smart ptrs
	std::vector<std::string> state_names_;
	std::map<std::string, std::map<Transition, std::string>> transition_map_;
	std::string log_path_;
	std::string text_file_;
	std::string dot_file_;

	bool isStateFound(std::string state);



}; // Class StateMachine


} // ns smach

#endif