
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "core/string/string_name.h"
#include "scene/main/node.h"

class HBStateMachine;

class HBStateMachineState : public Node {
	GDCLASS(HBStateMachineState, Node);

protected:
	HBStateMachine *state_machine = nullptr;

	virtual void enter(const Dictionary &p_args){};
	virtual void exit(){};
	virtual void physics_process(float p_delta){};
	virtual void process(float p_delta){};

	friend class HBStateMachine;
};

class HBStateMachine : public Node {
	GDCLASS(HBStateMachine, Node);

	ObjectID current_state_cache;

private:
	HBStateMachineState *_get_current_state();

protected:
	void _notification(int p_what);

public:
	void transition_to(const StringName &p_name, const Dictionary &p_args = {});

	HBStateMachine();
};

#endif // STATE_MACHINE_H
