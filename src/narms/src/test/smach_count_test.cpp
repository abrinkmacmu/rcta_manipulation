#include <state_machine.h>


// 1) define userdata and establish it in smach namespace


namespace smach {
struct ApplicationUserData {
	int n = 0;
};

ApplicationUserData UD;
}



// 2) define application states
class GraspingState: public smach::State {
public:
	GraspingState() {};
	smach::Transition executeState() {
		std::cout << "Executing grasing state\n";
		return smach::Success;
	}
};

class CountingState: public smach::State {
public:
	CountingState() {};
	smach::Transition executeState() {
		std::cout << "Executing counting state " << smach::UD.n <<  "\n";
		if (smach::UD.n < 10) {
			smach::UD.n++;
			return smach::Continue;
		} else {
			return smach::Complete;
		}
	}
};

// 3) register the states with the smach::stateMachine

int main(int argc, char *argv[]) {
	smach::UD.n = 0;
	GraspingState GS;
	CountingState CS;
	smach::StateMachine stateMachine;
	stateMachine.registerState("Grasping", &GS);
	stateMachine.registerState("CountUp", &CS);
	stateMachine.registerTransition("Grasping", smach::Success, "CountUp");
	stateMachine.registerTransition("CountUp", smach::Continue, "Grasping");
	stateMachine.registerTransition("CountUp", smach::Complete, smach::COMPLETED_TRANSITION);

// 4) run the stateMachine

	stateMachine.runStartingFromState("CountUp");

	return 0;
}