
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

	static void _bind_methods();

	GDVIRTUAL1(_enter, Dictionary);
	GDVIRTUAL1(_physics_process, float);
	GDVIRTUAL1(_process, float);
	GDVIRTUAL0(_exit);

	virtual void debug_ui_draw(){};

	Node *get_actor() const;

	friend class HBStateMachine;
};

class HBStateMachine : public Node {
	GDCLASS(HBStateMachine, Node);

	ObjectID current_state_cache;
	NodePath agent_node;
	ObjectID agent_node_cache;
	String default_state;

	void _update_agent_node_cache();

#ifdef DEBUG_ENABLED
	int selected_state = 0;
#endif

private:
	HBStateMachineState *_get_current_state();
	void _on_child_entered_tree(Node *p_child);

protected:
	void _notification(int p_what);
	Node *_get_actor();
	static void _bind_methods();

public:
	void transition_to(const StringName &p_name, const Dictionary &p_args = {});

	NodePath get_agent_node() const;
	void set_agent_node(const NodePath &p_actor_node);
	String get_default_state() const;
	void set_default_state(const String &p_default_state);

	HBStateMachine();
	~HBStateMachine();

	friend class HBStateMachineState;
};

#endif // STATE_MACHINE_H
