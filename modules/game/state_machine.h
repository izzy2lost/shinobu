
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "agent.h"
#include "core/object/object_id.h"
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

	Node *get_actor() const;

	friend class HBStateMachine;
};

class HBStateMachine : public Node {
	GDCLASS(HBStateMachine, Node);

	ObjectID current_state_cache;
	NodePath actor_node;
	ObjectID actor_node_cache;

	void _update_actor_node_cache();

private:
	HBStateMachineState *_get_current_state();

protected:
	void _notification(int p_what);
	Node *_get_actor();

public:
	void transition_to(const StringName &p_name, const Dictionary &p_args = {});

	HBStateMachine();

	NodePath get_actor_node() const;
	void set_actor_node(const NodePath &p_actor_node);
	friend class HBStateMachineState;
};

#endif // STATE_MACHINE_H
