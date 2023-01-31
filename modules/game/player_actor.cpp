#include "player_actor.h"
#include "scene/main/viewport.h"

Vector3 HBPlayerActor::get_input() const {
	Vector2 input = Input::get_singleton()->get_vector("move_left", "move_right", "move_forward", "move_backward");
	Vector3 input_3d_space = Vector3(input.x, 0, input.y);

	Camera3D *camera = get_viewport()->get_camera_3d();
	if (camera) {
		input_3d_space = camera->get_camera_transform().basis.xform(input_3d_space);
		input_3d_space.y = 0.0f;
		input_3d_space.normalize();
	}

	return input_3d_space;
}

HBPlayerActor::HBPlayerActor() :
		HBActor() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		Input::get_singleton()->set_mouse_mode(Input::MouseMode::MOUSE_MODE_CAPTURED);
	}
}
