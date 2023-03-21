#include "player_agent.h"
#include "scene/main/viewport.h"
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#include "modules/imgui/godot_imgui_macros.h"
#endif

HBPlayerAgent::HBPlayerAgent() :
		HBAgent() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		Input::get_singleton()->set_mouse_mode(Input::MouseMode::MOUSE_MODE_CAPTURED);
	}
}

HBAgent *HBPlayerAgentController::_get_agent() const {
	if (agent_node_cache.is_valid()) {
		return Object::cast_to<HBAgent>(ObjectDB::get_instance(agent_node_cache));
	} else {
		const_cast<HBPlayerAgentController *>(this)->_update_agent_node_cache();
		if (agent_node_cache.is_valid()) {
			return Object::cast_to<HBAgent>(ObjectDB::get_instance(agent_node_cache));
		}
	}

	return nullptr;
}

void HBPlayerAgentController::_update_agent_node_cache() {
	agent_node_cache = ObjectID();

	if (has_node(agent_node)) {
		Node *node = get_node(agent_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update agent cache: Node cannot be found!");

		// Ensure its a Node
		Node *nd = Object::cast_to<Node>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update agent cache: NodePath does not point to an HBAgent type node!");

		agent_node_cache = nd->get_instance_id();
	}
}

void HBPlayerAgentController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_agent_node", "agent_node"), &HBPlayerAgentController::set_agent_node);
	ClassDB::bind_method(D_METHOD("get_agent_node"), &HBPlayerAgentController::get_agent_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "agent_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "HBAgent"), "set_agent_node", "get_agent_node");
}

void HBPlayerAgentController::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PHYSICS_PROCESS: {
			HBAgent *agent = _get_agent();
			if (agent) {
				Input *input = Input::get_singleton();
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_RUN, input->is_action_pressed("move_run"));

				// Movement input
				Vector2 movement_input = Input::get_singleton()->get_vector("move_left", "move_right", "move_forward", "move_backward");
				Vector3 input_3d_space = Vector3(movement_input.x, 0, movement_input.y);

				Camera3D *camera = get_viewport()->get_camera_3d();
				if (camera) {
					input_3d_space = camera->get_camera_transform().basis.xform(input_3d_space);
					input_3d_space.y = 0.0f;
					input_3d_space.normalize();
				}

				agent->set_movement_input(input_3d_space);
			}
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
#ifdef DEBUG_ENABLED
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					ImDrawList *dl = ImGui::GetWindowDrawList();
					const float RADIUS = 50.0f;
					ImVec2 center_pos = ImGui::GetCursorScreenPos();
					center_pos.x += RADIUS;
					center_pos.y += RADIUS;
					dl->AddCircleFilled(center_pos, RADIUS, IM_COL32_BLACK);
					Vector2 movement_input = Input::get_singleton()->get_vector("move_left", "move_right", "move_forward", "move_backward");
					movement_input *= RADIUS;
					movement_input += Vector2(center_pos.x, center_pos.y);
					dl->AddLine(center_pos, ImVec2(movement_input.x, movement_input.y), IM_COL32_WHITE);
					ImGui::Dummy(ImVec2(RADIUS * 2, RADIUS * 2));
				}
				ImGui::End();
			}
#endif
		} break;
	}
}

NodePath HBPlayerAgentController::get_agent_node() const {
	return agent_node;
}

void HBPlayerAgentController::set_agent_node(const NodePath &p_agent_node) {
	agent_node = p_agent_node;
}

HBPlayerAgentController::HBPlayerAgentController() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process(true);
#ifdef DEBUG_ENABLED
		set_process_internal(true);
#endif
	}
	REGISTER_DEBUG(this);
}

HBPlayerAgentController::~HBPlayerAgentController() {
	UNREGISTER_DEBUG(this);
}
