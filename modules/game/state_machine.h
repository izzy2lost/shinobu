/**************************************************************************/
/*  state_machine.h                                                       */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

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
	HBStateMachineState *get_state(const String &p_state_name) const;

	HBStateMachine();
	~HBStateMachine();

	friend class HBStateMachineState;
};

#endif // STATE_MACHINE_H
