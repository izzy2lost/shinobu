#include "player_agent.h"
#include "scene/3d/node_3d.h"
#include "scene/main/viewport.h"
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#include "modules/imgui/godot_imgui_macros.h"
#endif

#include "core/config/project_settings.h"
#include "scene/resources/packed_scene.h"

HBPlayerAgent::HBPlayerAgent() :
		HBAgent() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		//Input::get_singleton()->set_mouse_mode(Input::MouseMode::MOUSE_MODE_CAPTURED);
	}
	GLOBAL_DEF_BASIC(PropertyInfo(Variant::STRING, "game/player/player_scene", PROPERTY_HINT_FILE, "*.tscn,*.scn,*.res"), "");
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
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
#endif
	switch (p_what) {
		case NOTIFICATION_PHYSICS_PROCESS: {
			HBAgent *agent = _get_agent();
			if (agent) {
				agent->flush_inputs();

				Input *input = Input::get_singleton();
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_RUN, input->is_action_pressed(SNAME("move_run")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN, input->is_action_pressed(SNAME("move_parkour_down")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_UP, input->is_action_pressed(SNAME("move_parkour_up")));

				// Movement input
				Vector2 movement_input = Input::get_singleton()->get_vector("move_left", "move_right", "move_forward", "move_backward");
				Camera3D *camera = get_viewport()->get_camera_3d();
				if (camera) {
					Quaternion camera_rot = camera->get_camera_transform().basis.get_rotation_quaternion();
					Vector3 curr_camera_forward = camera_rot.xform(Vector3(0.0f, 0.0f, -1.0f));
					Vector3 target_camera_forward = curr_camera_forward;
					target_camera_forward.y = 0.0f;
					target_camera_forward.normalize();
					// Guard against fully vertical camera aiming
					// Should never happen but...
					if (target_camera_forward.is_normalized()) {
						camera_rot = Quaternion(curr_camera_forward, target_camera_forward) * camera_rot;
						agent->set_movement_input_rotation(camera_rot);
					}
				}
				Vector3 input_3d_space = Vector3(movement_input.x, 0, movement_input.y);
				agent->set_movement_input(input_3d_space);
			}
		} break;
#ifdef DEBUG_ENABLED
		case NOTIFICATION_ENTER_TREE: {
			REGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_EXIT_TREE: {
			UNREGISTER_DEBUG(this);
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					Vector2 movement_input = Input::get_singleton()->get_vector("move_left", "move_right", "move_forward", "move_backward");
					GodotImGui::DrawJoystick(movement_input, 50.0f);
				}
				ImGui::End();
			}
		} break;
#endif
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
}

HBPlayerAgentController::~HBPlayerAgentController() {
}

void HBInfoPlayerStart::_editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) {
	String player_scene_path = GLOBAL_GET("game/player/player_scene");
	DEV_ASSERT(!player_scene_path.is_empty());
	Ref<PackedScene> packed_scene = ResourceLoader::load(player_scene_path);
	DEV_ASSERT(packed_scene.is_valid());
	Node *player_node = packed_scene->instantiate();
	HBPlayerAgent *player_agent_node = Object::cast_to<HBPlayerAgent>(player_node);
	DEV_ASSERT(player_agent_node);

	add_child(player_agent_node);
	player_agent_node->set_owner(get_owner());
}
