#include "agent_parkour.h"
#include "physics_layers.h"

#include "scene/resources/sphere_shape_3d.h"

void HBAgentParkourPoint::_update_collision_shape() {
	collision_shape_dirty = false;
	if (!collision_shape) {
		collision_shape = memnew(CollisionShape3D);
		shape.instantiate();
		add_child(collision_shape, false, INTERNAL_MODE_BACK);
	}
	shape->set_radius(0.1f);
	collision_shape->set_shape(shape);
}

void HBAgentParkourPoint::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PROCESS: {
			if (collision_shape_dirty) {
				_update_collision_shape();
				set_process(false);
			}
		} break;
	}
}

HBAgentParkourPoint::HBAgentParkourPoint() {
	set_collision_mask(0);
	set_collision_layer(HBPhysicsLayers::LAYER_PARKOUR_NODES);
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process(true);
	}
}