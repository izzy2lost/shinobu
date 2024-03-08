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
		case NOTIFICATION_READY: {
			debug_geo = memnew(HBDebugGeometry);
			add_child(debug_geo);
			debug_geo->set_as_top_level(true);
			debug_geo->set_global_transform(Transform3D());
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
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

	ClassDB::bind_method(D_METHOD("track_node"), &HBPlayerCameraArm::track_node);

	BIND_ENUM_CONSTANT(CameraTargetMode::BONE);
	BIND_ENUM_CONSTANT(CameraTargetMode::TRACK_NODES);
}

void HBPlayerCameraArm::_process_rotation(float p_delta) {
	Vector2 camera_look = Input::get_singleton()->get_vector("look_left", "look_right", "look_down", "look_up");
	Vector2 target_velocity = Vector2(Math::deg_to_rad(max_velocity), Math::deg_to_rad(max_velocity)) * camera_look;
	HBSprings::velocity_spring_vector2(velocity, acceleration, target_velocity, 0.025f, p_delta);
	Vector3 rotation = get_rotation();
	rotation.y = rotation.y - velocity.x * p_delta;
	rotation.x = CLAMP(rotation.x + velocity.y * p_delta, Math::deg_to_rad(min_pitch_degrees), Math::deg_to_rad(max_pitch_degrees));
	set_rotation(rotation);
}

void HBPlayerCameraArm::_track_bone(float p_delta) {
	Vector3 target_pos = get_target_position();
	target_pos.x = 0.0f;
	target_pos.z = 0.0f;
	Vector3 pos = get_global_position();
	pos.x = 0.0f;
	pos.z = 0.0f;
	HBSprings::critical_spring_damper_exact_vector3(pos, position_spring_velocity, target_pos, .25, p_delta);
	pos.x = get_target_position().x;
	pos.z = get_target_position().z;
	set_global_position(pos);
}

void HBPlayerCameraArm::_track_nodes(float p_delta) {
	// Find midpoint of all nodes
	debug_geo->set_global_transform(Transform3D());
	debug_geo->clear();
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

	for (size_t i = 0; i < tracked_nodes.size(); i++) {
		Node3D *node = Object::cast_to<Node3D>(ObjectDB::get_instance(tracked_nodes[i]));
		DEV_ASSERT(node);
		node_count++;
		midpoint += node->get_global_position();
	}

	if (node_count == 0) {
		return;
	}

	midpoint /= (float)node_count;

	Vector3 target_position = midpoint + Vector3(0.0f, height_offset, 0.0f);

	set_global_position(target_position);

	const Vector3 camera_forward = -get_global_basis().get_column(2);
	const Vector3 camera_right = get_global_basis().get_column(0);

	Vector3 camera_position = get_global_position() + get_global_basis().xform(Vector3(0.0f, 0.0f, get_length()));

	std::vector<float> differences;

	debug_geo->debug_sphere(camera_position, 0.05f, Color("GREEN"));
	for (size_t i = 0; i < tracked_nodes.size(); i++) {
		Node3D *node_to_track = Object::cast_to<Node3D>(ObjectDB::get_instance(tracked_nodes[i]));
		// Figure out if the node to track is to the right or to the left of the camera
		float side = camera_right.dot(node_to_track->get_global_position() - camera_position);

		if (side == 0.0) {
			print_line("NO SIDE!");
			continue;
		}

		// depending on the side we offset character radius a bit
		Vector3 tracked_pos = node_to_track->get_global_position(); // + camera_right * (SIGN(side));

		// Plane using character position and camera right
		Plane tracked_plane = Plane(camera_right, tracked_pos);

		// Now, we fire a ray from the right/leftmost side of the screen to the tracked position
		Vector2 ray_origin_screen_pos = get_window()->get_size();
		ray_origin_screen_pos.y = 0.0f;
		ray_origin_screen_pos.x *= side < 0.0f ? 0.0f : 1.0f;
		Vector3 ray_normal = camera->project_ray_normal(ray_origin_screen_pos);
		Vector3 ray_origin = camera->project_ray_origin(ray_origin_screen_pos);

		debug_geo->debug_line(ray_origin, ray_origin + ray_normal, Color("RED"));

		// Intersect the ray with the plane
		Vector3 intersection;
		if (!tracked_plane.intersects_ray(ray_origin, ray_normal, &intersection)) {
			print_line("NOINTERSECT");
			continue;
		}

		// Now project it on the camera forward and that is our offset
		Vector3 difference = (tracked_pos - intersection);
		float fwd_diff = difference.dot(camera_forward);
		differences.push_back(fwd_diff);

		print_line(i, fwd_diff);
	}

	if (differences.size() == 0) {
		return;
	}

	float min_diff = *std::min_element(differences.begin(), differences.end());
	Vector3 pos = get_global_position();
	pos += (camera_forward)*min_diff;
	pos -= (camera_forward)*forward_offset;
	set_global_position(pos);
}

void HBPlayerCameraArm::track_node(Node3D *p_node) {
	DEV_ASSERT(p_node);
	tracked_nodes.push_back(p_node->get_instance_id());
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

HBPlayerCameraArm::CameraTargetMode HBPlayerCameraArm::get_target_mode() const {
	return target_mode;
}

void HBPlayerCameraArm::set_target_mode(CameraTargetMode p_target_mode) {
	target_mode = p_target_mode;
}

void HBPlayerCameraArm::set_target_skeleton_path(const NodePath &p_skeleton_path) {
	target_skeleton_path = p_skeleton_path;
	_update_target_skeleton_node_cache();
}

NodePath HBPlayerCameraArm::get_target_skeleton_path() const {
	return target_skeleton_path;
}
