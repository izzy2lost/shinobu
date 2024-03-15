/**************************************************************************/
/*  epas_editor_camera.cpp                                                */
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

#include "epas_editor_camera.h"

#include "core/config/project_settings.h"
#include "scene/main/viewport.h"

void EPASEditorCamera::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PROCESS: {
			if (!rotating) {
				return;
			}
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

			if (rotating && !Input::get_singleton()->is_mouse_button_pressed(MouseButton::RIGHT)) {
				if (Input::get_singleton()->get_mouse_mode() == Input::MOUSE_MODE_CAPTURED) {
					Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_VISIBLE);
					rotating = false;
				}
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
	Ref<InputEventMouseButton> ev_mouse_but = p_event;
	if (ev_mouse_but.is_valid()) {
		if (ev_mouse_but->get_button_index() == MouseButton::RIGHT) {
			rotating = ev_mouse_but->is_pressed();
			if (rotating) {
				Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_CAPTURED);
			} else {
				Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_VISIBLE);
			}
		}
	}
}

EPASEditorCamera::EPASEditorCamera() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process_internal(true);
		set_process_unhandled_input(true);
	}
}