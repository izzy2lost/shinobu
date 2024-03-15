/**************************************************************************/
/*  player_camera_arm.cpp                                                 */
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

#include "player_camera_arm.h"
#include "game_main_loop.h"
#include "game_world.h"
#include "modules/game/player_agent.h"
#include "scene/main/viewport.h"
#include "springs.h"

void HBPlayerCameraArm::_process_inertialization(float p_delta) {
	if (inertialization_queued) {
		_inertialize(arm_transform, length, 0.75f, p_delta);
		inertialization_queued = false;
	}

	Vector3 pos_offset;
	if (pos_inertializer.is_valid()) {
		pos_offset = pos_inertializer->advance(p_delta);
		if (pos_inertializer->is_done()) {
			pos_inertializer.unref();
		}
	}

	Quaternion rot_offset;
	if (rot_inertializer.is_valid()) {
		rot_offset = rot_inertializer->advance(p_delta);
		if (rot_inertializer->is_done()) {
			rot_inertializer.unref();
		}
	}

	float len_offset = 0.0f;
	if (len_inertializer.is_valid()) {
		len_offset = len_inertializer->advance(p_delta);
		if (len_inertializer->is_done()) {
			len_inertializer.unref();
		}
	}

	set_global_position(arm_transform.origin + pos_offset);
	set_global_basis(arm_transform.basis.get_rotation_quaternion() * rot_offset);
	set_length(length + len_offset);

	prev_transform = get_global_transform();
	prev_length = get_length();
}

void HBPlayerCameraArm::_inertialize(Transform3D &p_new_transform, float p_new_len, float p_duration, float p_delta) {
	pos_inertializer = PositionInertializer::create(prev_transform.origin, get_global_position(), p_new_transform.origin, p_duration, p_delta);
	rot_inertializer = RotationInertializer::create(
			prev_transform.basis.get_rotation_quaternion(),
			get_global_basis().get_rotation_quaternion(),
			p_new_transform.basis.get_rotation_quaternion(),
			p_duration,
			p_delta);
	len_inertializer = ScalarInertializer::create(prev_length, get_length(), p_new_len, p_duration, p_delta);
}

void HBPlayerCameraArm::_update_target_skeleton_node_cache() {
	target_skeleton_node_cache = ObjectID();

	if (has_node(target_skeleton_path)) {
		Node *node = get_node(target_skeleton_path);
		ERR_FAIL_COND_MSG(!node, "Cannot update camera arm skeleton: Node cannot be found!");

		// Ensure its a Node3D
		Skeleton3D *nd = Object::cast_to<Skeleton3D>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update camera arm skeleton node cache: NodePath does not point to a Skeleton3D node!");

		target_skeleton_node_cache = nd->get_instance_id();
	}
}

Skeleton3D *HBPlayerCameraArm::get_target_skeleton() const {
	if (target_skeleton_node_cache.is_valid()) {
		return Object::cast_to<Skeleton3D>(ObjectDB::get_instance(target_skeleton_node_cache));
	} else {
		const_cast<HBPlayerCameraArm *>(this)->_update_target_skeleton_node_cache();
		if (target_skeleton_node_cache.is_valid()) {
			return Object::cast_to<Skeleton3D>(ObjectDB::get_instance(target_skeleton_node_cache));
		}
	}

	return nullptr;
}

Vector3 HBPlayerCameraArm::get_target_position() const {
	Node3D *parent = Object::cast_to<Node3D>(get_parent());
	Skeleton3D *skel = get_target_skeleton();

	Vector3 target_pos;

	if (int bone_idx; skel && (bone_idx = skel->find_bone(target_bone_name)) != -1) {
		target_pos = skel->to_global(skel->get_bone_global_pose(bone_idx).origin);
	} else if (parent) {
		target_pos = parent->get_global_position();
		target_pos += Vector3(0.0f, 0.75f, 0.0f);
	}

	return target_pos;
}

void HBPlayerCameraArm::unhandled_input(const Ref<InputEvent> &p_event) {
	const InputEventMouseMotion *ev_mouse_mot = Object::cast_to<InputEventMouseMotion>(*p_event);

	if (ev_mouse_mot) {
		if (Input::get_singleton()->get_mouse_mode() != Input::MOUSE_MODE_CAPTURED) {
			return;
		}
		Vector2 relative = ev_mouse_mot->get_relative();
		relative = relative / get_viewport()->get_visible_rect().size.x;
		// TODO: make this user editable
		real_t sensitivity = 175.0f;
		sensitivity = Math::deg_to_rad(sensitivity);

		relative *= sensitivity;

		Vector3 rotation = arm_transform.basis.get_euler();
		rotation.y += -relative.x;
		rotation.x += -relative.y;

		rotation.x = CLAMP(rotation.x, Math::deg_to_rad(min_pitch_degrees), Math::deg_to_rad(max_pitch_degrees));

		arm_transform.basis = Basis::from_euler(rotation);

		velocity = Vector2();
	}
}

void HBPlayerCameraArm::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PROCESS: {
			float delta = get_physics_process_delta_time();
			_process_rotation(delta);

			switch (target_mode) {
				case BONE: {
					_track_bone(delta);
				} break;
				case TRACK_NODES: {
					_track_nodes(delta);
				} break;
			}

			_process_inertialization(delta);

		} break;
		case HBGameWorld::NOTIFICATION_HB_ENTER_GAME_WORLD: {
			HBGameMainLoop::get_singleton()->get_game_world()->get_game_world_state()->connect(SNAME("agent_entered_combat"), callable_mp(this, &HBPlayerCameraArm::_on_agent_entered_combat));
			HBGameMainLoop::get_singleton()->get_game_world()->get_game_world_state()->connect(SNAME("agent_exited_combat"), callable_mp(this, &HBPlayerCameraArm::_on_agent_exited_combat));
		} break;
		case HBGameWorld::NOTIFICATION_HB_EXIT_GAME_WORLD: {
			HBGameMainLoop::get_singleton()->get_game_world()->get_game_world_state()->disconnect(SNAME("agent_entered_combat"), callable_mp(this, &HBPlayerCameraArm::_on_agent_entered_combat));
			HBGameMainLoop::get_singleton()->get_game_world()->get_game_world_state()->disconnect(SNAME("agent_exited_combat"), callable_mp(this, &HBPlayerCameraArm::_on_agent_exited_combat));

		} break;
		case NOTIFICATION_PARENTED: {
			if (HBPlayerAgent *player = Object::cast_to<HBPlayerAgent>(get_parent()); player) {
				player_parent = player;
			}
		} break;
		case NOTIFICATION_UNPARENTED: {
			player_parent = nullptr;
		} break;
	}
}

void HBPlayerCameraArm::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_target_skeleton_path", "target_skeleton_path"), &HBPlayerCameraArm::set_target_skeleton_path);
	ClassDB::bind_method(D_METHOD("get_target_skeleton_path"), &HBPlayerCameraArm::get_target_skeleton_path);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "skeleton_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Skeleton3D"), "set_target_skeleton_path", "get_target_skeleton_path");

	ClassDB::bind_method(D_METHOD("set_target_bone_name", "target_bone_name"), &HBPlayerCameraArm::set_target_bone_name);
	ClassDB::bind_method(D_METHOD("get_target_bone_name"), &HBPlayerCameraArm::get_target_bone_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "target_bone_name"), "set_target_bone_name", "get_target_bone_name");

	ClassDB::bind_method(D_METHOD("set_target_mode", "target_mode"), &HBPlayerCameraArm::set_target_mode);
	ClassDB::bind_method(D_METHOD("get_target_mode"), &HBPlayerCameraArm::get_target_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "target_mode", PROPERTY_HINT_ENUM, "Bone,Track nodes"), "set_target_mode", "get_target_mode");

	ClassDB::bind_method(D_METHOD("track_node", "node"), &HBPlayerCameraArm::track_node);

	BIND_ENUM_CONSTANT(CameraTargetMode::BONE);
	BIND_ENUM_CONSTANT(CameraTargetMode::TRACK_NODES);

	ClassDB::bind_method(D_METHOD("set_node_tracking_radius", "tracking_radius"), &HBPlayerCameraArm::set_node_tracking_radius);
	ClassDB::bind_method(D_METHOD("get_node_tracking_radius"), &HBPlayerCameraArm::get_node_tracking_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "node_tracking_radius"), "set_node_tracking_radius", "get_node_tracking_radius");

	ClassDB::bind_method(D_METHOD("set_transition_duration", "transition_duration"), &HBPlayerCameraArm::set_transition_duration);
	ClassDB::bind_method(D_METHOD("get_transition_duration"), &HBPlayerCameraArm::get_transition_duration);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "transition_duration"), "set_transition_duration", "get_transition_duration");
}

void HBPlayerCameraArm::_process_rotation(float p_delta) {
	Vector2 camera_look = Input::get_singleton()->get_vector("look_left", "look_right", "look_down", "look_up");
	Vector2 target_velocity = Vector2(Math::deg_to_rad(max_velocity), Math::deg_to_rad(max_velocity)) * camera_look;
	HBSprings::velocity_spring_vector2(velocity, acceleration, target_velocity, 0.025f, p_delta);
	Vector3 rotation = arm_transform.basis.get_euler();
	rotation.y = rotation.y - velocity.x * p_delta;
	rotation.x = CLAMP(rotation.x + velocity.y * p_delta, Math::deg_to_rad(min_pitch_degrees), Math::deg_to_rad(max_pitch_degrees));
	arm_transform.basis = Basis::from_euler(rotation);
}

void HBPlayerCameraArm::_track_bone(float p_delta) {
	length = target_length;

	Vector3 target_pos = get_target_position();
	target_pos.x = 0.0f;
	target_pos.z = 0.0f;
	Vector3 pos = arm_transform.origin;
	pos.x = 0.0f;
	pos.z = 0.0f;
	HBSprings::critical_spring_damper_exact_vector3(pos, position_spring_velocity, target_pos, .25, p_delta);
	pos.x = get_target_position().x;
	pos.z = get_target_position().z;
	arm_transform.origin = pos;
}

void HBPlayerCameraArm::_track_nodes(float p_delta) {
	if (tracked_nodes_update_queued) {
		tracked_nodes = HBGameMainLoop::get_singleton()->get_game_world()->get_game_world_state()->get_agents_in_combat();
		tracked_nodes_update_queued = false;
	}

	// Find midpoint of all nodes
	Camera3D *camera = nullptr;
	for (int i = get_child_count() - 1; 0 <= i; --i) {
		camera = Object::cast_to<Camera3D>(get_child(i));
		if (camera) {
			break;
		}
	}

	DEV_ASSERT(camera);

	Vector3 midpoint = Vector3();
	int node_count = 0;

	for (ObjectID obj_id : tracked_nodes) {
		Node3D *node = Object::cast_to<Node3D>(ObjectDB::get_instance(obj_id));
		DEV_ASSERT(node);
		node_count++;
		midpoint += node->get_global_position();
	}

	if (node_count == 0) {
		return;
	}

	midpoint /= (float)node_count;
	midpoint.y += height_offset;

	Vector3 target_position = midpoint;

	float max_radius = 0.0f;
	for (ObjectID obj_id : tracked_nodes) {
		Node3D *node_to_track = Object::cast_to<Node3D>(ObjectDB::get_instance(obj_id));
		max_radius = MAX(max_radius, midpoint.distance_to(node_to_track->get_global_position()));
	}

	const float half_fov = Math::deg_to_rad(camera->get_fov() * 0.5f);

	arm_transform.origin = target_position;

	length = max_radius / sin(half_fov);
	length = MAX(length, target_length);
}

void HBPlayerCameraArm::_on_agent_entered_combat(HBAgent *p_agent) {
	if (p_agent == player_parent) {
		set_target_mode(CameraTargetMode::TRACK_NODES);
	}
	if (target_mode == HBPlayerCameraArm::TRACK_NODES) {
		inertialization_queued = true;
	}
	tracked_nodes_update_queued = true;
}

void HBPlayerCameraArm::_on_agent_exited_combat(HBAgent *p_agent) {
	if (p_agent == player_parent) {
		set_target_mode(CameraTargetMode::BONE);
	}
	tracked_nodes_update_queued = true;
}

void HBPlayerCameraArm::track_node(Node3D *p_node) {
	DEV_ASSERT(p_node);
	tracked_nodes.insert(p_node->get_instance_id());
}

HBPlayerCameraArm::HBPlayerCameraArm() :
		SpringArm3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process_unhandled_input(true);
		set_process_internal(true);
		set_as_top_level(true);
	}
}

StringName HBPlayerCameraArm::get_target_bone_name() const {
	return target_bone_name;
}

void HBPlayerCameraArm::set_target_bone_name(const StringName &p_target_bone_name) {
	target_bone_name = p_target_bone_name;
}

HBPlayerCameraArm::CameraTargetMode HBPlayerCameraArm::get_target_mode() const {
	return target_mode;
}

void HBPlayerCameraArm::set_target_mode(CameraTargetMode p_target_mode) {
	target_mode = p_target_mode;
	inertialization_queued = true;
}

void HBPlayerCameraArm::set_target_skeleton_path(const NodePath &p_skeleton_path) {
	target_skeleton_path = p_skeleton_path;
	_update_target_skeleton_node_cache();
}

NodePath HBPlayerCameraArm::get_target_skeleton_path() const {
	return target_skeleton_path;
}
