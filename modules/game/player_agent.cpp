/**************************************************************************/
/*  player_agent.cpp                                                      */
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

#include "player_agent.h"
#include "modules/game/game_main_loop.h"
#include "modules/game/game_world.h"
#include "scene/main/viewport.h"
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#include "modules/imgui/godot_imgui_macros.h"
#endif

#include "core/config/project_settings.h"
#include "scene/resources/packed_scene.h"

void HBPlayerAgent::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (!Engine::get_singleton()->is_editor_hint()) {
				HBGameWorld *gw = Object::cast_to<HBGameMainLoop>(get_tree())->get_game_world();
				gw->set_player(this);
			}
		} break;
	}
}

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
		case NOTIFICATION_PARENTED: {
			HBAgent *parent = Object::cast_to<HBAgent>(get_parent());
			if (parent) {
				parent->set_is_player_controlled(true);
			}
		} break;
		case NOTIFICATION_PHYSICS_PROCESS: {
			HBAgent *agent = _get_agent();
			if (agent) {
				agent->flush_inputs();

				Input *input = Input::get_singleton();
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_RUN, input->is_action_pressed(SNAME("move_run")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_DOWN, input->is_action_pressed(SNAME("move_parkour_down")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_PARKOUR_UP, input->is_action_pressed(SNAME("move_parkour_up")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_TARGET, input->is_action_pressed(SNAME("target")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_ATTACK, input->is_action_pressed(SNAME("attack")));
				agent->set_input_action_state(HBAgent::AgentInputAction::INPUT_ACTION_PARRY, input->is_action_pressed(SNAME("parry")));

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

void HBInfoPlayerStart::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			add_to_group("info_player_start");
		} break;
	}
}

void HBInfoPlayerStart::_editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) {
}
