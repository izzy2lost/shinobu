#include "epas_editor_camera.h"

#include "core/config/project_settings.h"
#include "scene/main/viewport.h"

void EPASEditorCamera::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PROCESS: {
			Transform3D global_trf = get_global_transform();
			Vector3 loc_trf;
			if (Input::get_singleton()->is_key_pressed(Key::W)) {
				loc_trf.z -= 1;
			} else if (Input::get_singleton()->is_key_pressed(Key::S)) {
				loc_trf.z += 1;
			}
			if (Input::get_singleton()->is_key_pressed(Key::D)) {
				loc_trf.x += 1;
			} else if (Input::get_singleton()->is_key_pressed(Key::A)) {
				loc_trf.x -= 1;
			}
			if (Input::get_singleton()->is_key_pressed(Key::E)) {
				loc_trf.y += 1;
			} else if (Input::get_singleton()->is_key_pressed(Key::Q)) {
				loc_trf.y -= 1;
			}

			loc_trf.normalize();

			float fly_speed_multiplier = 1.0f;
			if (Input::get_singleton()->is_key_pressed(Key::SHIFT)) {
				fly_speed_multiplier = 2.0f;
			}
			loc_trf *= get_process_delta_time() * fly_speed_multiplier;

			Vector3 final_loc_trf;
			final_loc_trf += loc_trf.z * global_trf.basis.get_column(Vector3::AXIS_Z);
			final_loc_trf += loc_trf.x * global_trf.basis.get_column(Vector3::AXIS_X);
			final_loc_trf += loc_trf.y * global_trf.basis.get_column(Vector3::AXIS_Y);
			global_trf.origin += final_loc_trf;

			set_global_transform(global_trf);

			if (Input::get_singleton()->is_mouse_button_pressed(MouseButton::RIGHT)) {
				Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_CAPTURED);
			} else {
				Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_VISIBLE);
			}
		} break;
	}
}

void EPASEditorCamera::unhandled_input(const Ref<InputEvent> &p_event) {
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

		rotation.x = CLAMP(rotation.x, Math::deg_to_rad(-65.0f), Math::deg_to_rad(65.0f));
		set_rotation(rotation);
	}
}

EPASEditorCamera::EPASEditorCamera() {
	set_process_internal(true);
	set_process_unhandled_input(true);
}