#include "player_camera_arm.h"
#include "scene/main/viewport.h"

void HBPlayerCameraArm::unhandled_input(const Ref<InputEvent> &p_event) {
	const InputEventMouseMotion *ev_mouse_mot = Object::cast_to<InputEventMouseMotion>(*p_event);
	if (ev_mouse_mot) {
		Vector2 relative = ev_mouse_mot->get_relative();
		relative = relative / get_viewport()->get_visible_rect().size.x;
		real_t sensitivity = GLOBAL_GET("game/mouse_sensitivity");
		sensitivity = Math::deg_to_rad(sensitivity);

		relative *= sensitivity;

		Vector3 rotation = get_rotation();
		rotation.y += -relative.x;
		rotation.x += -relative.y;

		rotation.x = CLAMP(rotation.x, Math::deg_to_rad(-65.0f), Math::deg_to_rad(65.0f));

		set_rotation(rotation);
	}
}

void HBPlayerCameraArm::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
		} break;
	}
}

HBPlayerCameraArm::HBPlayerCameraArm() :
		SpringArm3D() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process_internal(true);
		set_process_unhandled_input(true);
	}
}
