#include "state_machine.h"

void HBStateMachine::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PHYSICS_PROCESS: {
			HBStateMachineState *current_state = _get_current_state();
			if (current_state) {
				current_state->physics_process(get_physics_process_delta_time());
			}
		} break;
		case NOTIFICATION_PROCESS: {
			HBStateMachineState *current_state = _get_current_state();
			if (current_state) {
				current_state->process(get_process_delta_time());
			}
		} break;
	}
}

Node *HBStateMachine::_get_actor() {
	if (actor_node_cache.is_valid()) {
		return Object::cast_to<Node>(ObjectDB::get_instance(actor_node_cache));
	} else {
		_update_actor_node_cache();
		if (actor_node_cache.is_valid()) {
			return Object::cast_to<Node>(ObjectDB::get_instance(actor_node_cache));
		}
	}

	return nullptr;
}

void HBStateMachine::_update_actor_node_cache() {
	actor_node_cache = ObjectID();

	if (has_node(actor_node)) {
		Node *node = get_node(actor_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor cache: Node cannot be found!");

		// Ensure its a Node
		Node *nd = Object::cast_to<Node>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor cache: NodePath does not point to a Node type node!");

		actor_node_cache = nd->get_instance_id();
	}
}

HBStateMachineState *HBStateMachine::_get_current_state() {
	if (current_state_cache.is_valid()) {
		return Object::cast_to<HBStateMachineState>(ObjectDB::get_instance(current_state_cache));
	}

	current_state_cache = ObjectID();

	return nullptr;
}

void HBStateMachine::transition_to(const StringName &p_name, const Dictionary &p_args) {
	HBStateMachineState *current_state = _get_current_state();
	if (current_state) {
		current_state->exit();
		current_state = nullptr;
	}
	current_state = Object::cast_to<HBStateMachineState>(get_node(String(p_name)));
	ERR_FAIL_COND_MSG(!current_state, "State machine state not found: " + p_name);
	current_state->enter(p_args);
	current_state_cache = current_state->get_instance_id();
}

HBStateMachine::HBStateMachine() {
	set_physics_process(true);
	set_process(true);
}

NodePath HBStateMachine::get_actor_node() const {
	return actor_node;
}

void HBStateMachine::set_actor_node(const NodePath &p_actor_node) {
	actor_node = p_actor_node;
}

Node *HBStateMachineState::get_actor() const {
	return state_machine->_get_actor();
}
