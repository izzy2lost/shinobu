#include "player_camera_arm.h"
#include "scene/main/viewport.h"
#include "utils.h"

void HBPlayerCameraArm::unhandled_input(const Ref<InputEvent> &p_event) {
	const InputEventMouseMotion *ev_mouse_mot = Object::cast_to<InputEventMouseMotion>(*p_event);
	if (ev_mouse_mot) {
		if (Input::get_singleton()->get_mouse_mode() != Input::MOUSE_MODE_CAPTURED) {
			return;
		}
		Vector2 relative = ev_mouse_mot->get_relative();
		relative = relative / get_viewport()->get_visible_rect().size.x;
		real_t sensitivity = GLOBAL_GET("game/mouse_sensitivity");
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
		case NOTIFICATION_INTERNAL_PROCESS: {
			Vector2 camera_look = Input::get_singleton()->get_vector("look_left", "look_right", "look_down", "look_up");
			float delta = get_process_delta_time();
			Vector2 target_velocity = Vector2(Math::deg_to_rad(max_velocity), Math::deg_to_rad(max_velocity)) * camera_look;
			HBUtils::velocity_spring_vector2(velocity, acceleration, target_velocity, 0.025f, delta);
			Vector3 rotation = get_rotation();
			rotation.y = rotation.y - velocity.x * delta;
			rotation.x = CLAMP(rotation.x + velocity.y * delta, Math::deg_to_rad(min_pitch_degrees), Math::deg_to_rad(max_pitch_degrees));
			set_rotation(rotation);

			Node3D *parent = Object::cast_to<Node3D>(get_parent());
			if (parent) {
				Vector3 pos = get_global_position();
				HBUtils::critical_spring_damper_exact_vector3(pos, position_spring_velocity, parent->get_global_position() + Vector3(0.0f, 0.75f, 0.0f), .15, get_process_delta_time());
				set_global_position(pos);
			}

		} break;
	}
}

HBPlayerCameraArm::HBPlayerCameraArm() :
		SpringArm3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process_internal(true);
		set_process_unhandled_input(true);
		set_process_internal(true);
		set_as_top_level(true);
	}
}
