#include <state_machine.h>


// 1) define userdata and establish it in smach namespace


namespace smach {
	struct ApplicationUserData {
		int n = 0;
	};

	ApplicationUserData UD;
}
	smach::UD.n = 0;


// 2) define application states
class GraspingState: public smach::State {
public:
	GraspingState(){};
	smach::Transition executeState() {

		return smach::Success;
	}
};

class CountingState: public smach::State {
public:
	CountingState(){};
	smach::Transition executeState() {
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
	GraspingState GS;
	CountingState CS;

	smach::stateMachine.registerState("Grasping", &GS);
	smach::stateMachine.registerState("CountUp", &CS);
	smach::stateMachine.registerTransition("Grasping", smach::Success, "CountUp");
	smach::stateMachine.registerTransition("CountUp", smach::Success, "Grasping");
	smach::stateMachine.registerTransition("CountUp", smach::Complete, smach::COMPLETED_TRANSITION);

// 4) run the stateMachine

	smach::stateMachine.runStartingFromState("CountUp");

	return 0;
}