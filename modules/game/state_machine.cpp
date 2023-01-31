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
