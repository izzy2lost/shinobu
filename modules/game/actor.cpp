#include "actor.h"
#include "utils.h"

#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#endif

#include "modules/imgui/godot_imgui_macros.h"

#include "core/config/project_settings.h"

HBActor::HBActor() :
		CharacterBody3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process_internal(true);
		set_process_internal(true);
	}

	REGISTER_DEBUG(this);
}

HBActor::~HBActor() {
	UNREGISTER_DEBUG(this);
}

void HBActor::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_input"), &HBActor::get_input);
	ClassDB::bind_method(D_METHOD("set_graphics_node", "path"), &HBActor::set_graphics_node);
	ClassDB::bind_method(D_METHOD("get_graphics_node"), &HBActor::get_graphics_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "graphics_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_graphics_node", "get_graphics_node");
	ClassDB::bind_method(D_METHOD("set_tilt_node", "path"), &HBActor::set_tilt_node);
	ClassDB::bind_method(D_METHOD("get_tilt_node"), &HBActor::get_tilt_node);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "tilt_node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_tilt_node", "get_tilt_node");
}

void HBActor::_rotate_towards_velocity(float p_delta) {
	Vector3 input_vector = get_input();
	Node3D *gn = _get_graphics_node();

	if (gn) {
		// Handle horizontal rotation of the actor
		if (input_vector.length() > 0.0) {
			Vector3 desired_look_normal = get_velocity();
			desired_look_normal.y = 0.0f;
			desired_look_normal.normalize();
			if (desired_look_normal.length() > 0.0f) {
				Transform3D global_trf = gn->get_global_transform();
				Vector3 current_forward = -global_trf.get_basis().get_column(Vector3::AXIS_Z);
				current_forward.y = 0.0f;
				current_forward.normalize();

				real_t rotation_speed = Math::deg_to_rad((real_t)GLOBAL_GET("game/player/graphics_rotation_speed"));

				HBUtils::rotate_normal_towards(current_forward, desired_look_normal, p_delta * rotation_speed);

				gn->look_at(gn->get_global_transform().origin + current_forward);
			}
		}
	}
}

void HBActor::_tilt_towards_acceleration(float p_delta) {
	Node3D *tn = _get_tilt_node();
	if (tn) {
		// Handle acceleration tilting of the actor
		Vector3 accel = velocity_spring_acceleration;
		accel.y = 0.0f;
		Vector3 tilt_axis = accel.cross(Vector3(0.0f, 1.0f, 0.0));
		tilt_axis.normalize();
		real_t angle = Math::deg_to_rad(CLAMP(-accel.length() / 40.0f, -1.0f, 1.0f) * 12.5f);

		Transform3D global_trf = tn->get_global_transform();

		if (!tilt_axis.is_normalized()) {
			tilt_axis = global_trf.get_basis().get_column(Vector3::AXIS_X);
		}

		Quaternion rotation_goal(tilt_axis, angle);

		Quaternion current_rot = tn->get_global_transform().basis.get_rotation_quaternion();
		rotation_goal = rotation_goal * tn->get_parent_node_3d()->get_global_transform().get_basis().get_rotation_quaternion();
		HBUtils::simple_spring_damper_exact_quat(current_rot, tilt_spring_velocity, rotation_goal, tilt_spring_halflife, p_delta);
		global_trf.basis = Basis(current_rot);
		tn->set_global_transform(global_trf);
	}
}

void HBActor::_physics_process(float p_delta) {
	Vector3 vel = get_velocity();
	switch (movement_mode) {
		case MovementMode::MOVE_GROUNDED: {
			Vector3 input_vector = get_input();
			Vector3 desired_velocity = input_vector * move_velocity;
			HBUtils::velocity_spring_vector3(vel, velocity_spring_acceleration, desired_velocity, velocity_spring_halflife, p_delta);
		} break;
	}

	vel += gravity * p_delta;

	set_velocity(vel);

	_rotate_towards_velocity(p_delta);
	_tilt_towards_acceleration(p_delta);

	move_and_slide();
}

void HBActor::_update_graphics_node_cache() {
	graphics_node_cache = ObjectID();

	if (has_node(graphics_node)) {
		Node *node = get_node(graphics_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor graphics cache: Node cannot be found!");

		// Ensure its a Node3D
		Node3D *nd = Object::cast_to<Node3D>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor graphics cache: NodePath does not point to a Node3D node!");

		graphics_node_cache = nd->get_instance_id();
	}
}

void HBActor::_update_tilt_node_cache() {
	tilt_node_cache = ObjectID();

	if (has_node(tilt_node)) {
		Node *node = get_node(tilt_node);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor tilt node cache: Node cannot be found!");

		// Ensure its a Node3D
		Node3D *nd = Object::cast_to<Node3D>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update actor tilt node cache: Node3D Nodepath does not point to a Node3D node!");

		tilt_node_cache = nd->get_instance_id();
	}
}

void HBActor::set_graphics_node(NodePath p_path) {
	graphics_node = p_path;
	_update_graphics_node_cache();
}

NodePath HBActor::get_graphics_node() const {
	return graphics_node;
}

NodePath HBActor::get_tilt_node() const {
	return tilt_node;
}

void HBActor::set_tilt_node(NodePath p_path) {
	tilt_node = p_path;
	_update_tilt_node_cache();
}

Node3D *HBActor::_get_graphics_node() {
	if (graphics_node_cache.is_valid()) {
		return Object::cast_to<Node3D>(ObjectDB::get_instance(graphics_node_cache));
	} else {
		_update_graphics_node_cache();
		if (graphics_node_cache.is_valid()) {
			return Object::cast_to<Node3D>(ObjectDB::get_instance(graphics_node_cache));
		}
	}

	return nullptr;
}

Node3D *HBActor::_get_tilt_node() {
	if (tilt_node_cache.is_valid()) {
		return Object::cast_to<Node3D>(ObjectDB::get_instance(tilt_node_cache));
	} else {
		_update_tilt_node_cache();
		if (tilt_node_cache.is_valid()) {
			return Object::cast_to<Node3D>(ObjectDB::get_instance(tilt_node_cache));
		}
	}

	return nullptr;
}

void HBActor::set_movement_mode(MovementMode p_movement_mode) {
	movement_mode = p_movement_mode;
}

HBActor::MovementMode HBActor::get_movement_mode() const {
	return movement_mode;
}

void HBActor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			_physics_process(get_physics_process_delta_time());
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
#ifdef DEBUG_ENABLED
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					ImGui::Text("Velocity %s", String(get_global_position()).utf8().ptr());
					ImGui::Separator();
					ImGui::InputFloat("Velocity Spring Half-life", &velocity_spring_halflife);
					ImGui::Text("Tilt spring velocity %s", String(get_global_position()).utf8().ptr());
					ImGui::InputFloat("Tilt Spring Half-life", &tilt_spring_halflife);
				}
				ImGui::End();
			}
#endif
		} break;
	}
}
