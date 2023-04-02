#include "state_machine.h"
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#include "modules/imgui/godot_imgui_macros.h"
#endif

void HBStateMachine::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (!Engine::get_singleton()->is_editor_hint() && !default_state.is_empty()) {
				Node *actor = _get_actor();
				ERR_FAIL_COND_MSG(actor == nullptr, "No actor was given to the state machine");
				actor->connect("ready", callable_mp(this, &HBStateMachine::transition_to).bind(default_state, Dictionary()), CONNECT_ONE_SHOT);
			}
		} break;
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
#ifdef DEBUG_ENABLED
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim->is_debug_enabled(this) && gim->begin_debug_window(this)) {
				HBStateMachineState *selected_state_ptr = nullptr;
				if (ImGui::BeginListBox("##States", ImVec2(250, 100))) {
					for (int i = 0; i < get_child_count(); i++) {
						HBStateMachineState *state = Object::cast_to<HBStateMachineState>(get_child(i));
						if (!state) {
							continue;
						}
						String state_name = state->get_name();
						if (ImGui::Selectable(state_name.utf8().get_data(), state->get_index() == selected_state)) {
							selected_state = state->get_index();
						}
						if (state->get_index() == selected_state) {
							selected_state_ptr = state;
						}
					}
					ImGui::EndListBox();
				}
				ImGui::SameLine();
				if (selected_state_ptr != nullptr) {
					selected_state_ptr->debug_draw();
				}
			}
			ImGui::End();
#endif
		} break;
	}
}

Node *HBStateMachine::_get_actor() {
	if (agent_node_cache.is_valid()) {
		return Object::cast_to<Node>(ObjectDB::get_instance(agent_node_cache));
	} else {
		_update_agent_node_cache();
		if (agent_node_cache.is_valid()) {
			return Object::cast_to<Node>(ObjectDB::get_instance(agent_node_cache));
		}
	}

	return nullptr;
}

void HBStateMachine::_bind_methods() {
	ClassDB::bind_method(D_METHOD("transition_to", "state", "args"), &HBStateMachine::transition_to);

	ClassDB::bind_method(D_METHOD("set_agent_node", "agent_node"), &HBStateMachine::set_agent_node);
	ClassDB::bind_method(D_METHOD("get_agent_node"), &HBStateMachine::get_agent_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "agent_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "HBAgent"), "set_agent_node", "get_agent_node");

	ClassDB::bind_method(D_METHOD("set_default_state", "default_state"), &HBStateMachine::set_default_state);
	ClassDB::bind_method(D_METHOD("get_default_state"), &HBStateMachine::get_default_state);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "default_state"), "set_default_state", "get_default_state");
}

void HBStateMachine::_update_agent_node_cache() {
	agent_node_cache = ObjectID();

	if (has_node(agent_node)) {
		Node *node = get_node(agent_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor cache: Node cannot be found!");

		// Ensure its a Node
		Node *nd = Object::cast_to<Node>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor cache: NodePath does not point to a Node type node!");

		agent_node_cache = nd->get_instance_id();
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
	current_state->state_machine = this;
	current_state->enter(p_args);
	current_state_cache = current_state->get_instance_id();
}

HBStateMachine::HBStateMachine() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process(true);
		set_process(true);
		REGISTER_DEBUG(this);
	}
}

HBStateMachine::~HBStateMachine() {
	UNREGISTER_DEBUG(this);
}

NodePath HBStateMachine::get_agent_node() const {
	return agent_node;
}

void HBStateMachine::set_agent_node(const NodePath &p_agent_node) {
	agent_node = p_agent_node;
}

String HBStateMachine::get_default_state() const {
	return default_state;
}

void HBStateMachine::set_default_state(const String &p_default_state) {
	default_state = p_default_state;
}

void HBStateMachineState::_bind_methods() {
	GDVIRTUAL_BIND(_enter, "args");
	GDVIRTUAL_BIND(_physics_process, "delta");
	GDVIRTUAL_BIND(_process, "delta");
	GDVIRTUAL_BIND(_exit);
}

Node *HBStateMachineState::get_actor() const {
	return state_machine->_get_actor();
}
