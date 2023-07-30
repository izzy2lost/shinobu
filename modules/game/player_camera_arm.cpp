#include "player_camera_arm.h"
#include "scene/main/viewport.h"
#include "springs.h"

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

		Vector3 rotation = get_rotation();
		rotation.y += -relative.x;
		rotation.x += -relative.y;

		rotation.x = CLAMP(rotation.x, Math::deg_to_rad(min_pitch_degrees), Math::deg_to_rad(max_pitch_degrees));

		set_rotation(rotation);
		velocity = Vector2();
	}
}

void HBPlayerCameraArm::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			Vector2 camera_look = Input::get_singleton()->get_vector("look_left", "look_right", "look_down", "look_up");
			float delta = get_physics_process_delta_time();
			Vector2 target_velocity = Vector2(Math::deg_to_rad(max_velocity), Math::deg_to_rad(max_velocity)) * camera_look;
			HBSprings::velocity_spring_vector2(velocity, acceleration, target_velocity, 0.025f, delta);
			Vector3 rotation = get_rotation();
			rotation.y = rotation.y - velocity.x * delta;
			rotation.x = CLAMP(rotation.x + velocity.y * delta, Math::deg_to_rad(min_pitch_degrees), Math::deg_to_rad(max_pitch_degrees));
			set_rotation(rotation);

			Vector3 target_pos = get_target_position();
			Vector3 pos = get_global_position();
			HBSprings::critical_spring_damper_exact_vector3(pos, position_spring_velocity, target_pos, .25, delta);
			set_global_position(pos);

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
}

HBPlayerCameraArm::HBPlayerCameraArm() :
		SpringArm3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process_unhandled_input(true);
		set_physics_process_internal(true);
		set_as_top_level(true);
	}
}

StringName HBPlayerCameraArm::get_target_bone_name() const {
	return target_bone_name;
}

void HBPlayerCameraArm::set_target_bone_name(const StringName &p_target_bone_name) {
	target_bone_name = p_target_bone_name;
}

void HBPlayerCameraArm::set_target_skeleton_path(const NodePath &p_skeleton_path) {
	target_skeleton_path = p_skeleton_path;
	_update_target_skeleton_node_cache();
}

NodePath HBPlayerCameraArm::get_target_skeleton_path() const {
	return target_skeleton_path;
}
