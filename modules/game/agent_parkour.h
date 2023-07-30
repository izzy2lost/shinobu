#ifndef AGENT_PARKOUR_H
#define AGENT_PARKOUR_H

#include "agent.h"
#include "debug_geometry.h"
#include "physics_layers.h"
#include "scene/3d/collision_shape_3d.h"
#include "scene/3d/physics_body_3d.h"
#include "scene/resources/box_shape_3d.h"
#include "scene/resources/sphere_shape_3d.h"

class HBAgentParkourPoint : public StaticBody3D {
	GDCLASS(HBAgentParkourPoint, StaticBody3D);
	bool collision_shape_dirty = true;
	Ref<SphereShape3D> shape;
	CollisionShape3D *collision_shape = nullptr;
	void _update_collision_shape();

protected:
	void _notification(int p_what);

public:
	HBAgentParkourPoint();
};

namespace ParkourAutojump {
static bool is_wall(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

static bool is_floor(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}
// Generic autojump solver
struct AgentParkourAutojumpData {
	bool found = false;
	Vector3 start;
	Vector3 end;
	Vector3 end_normal;
	float parabola_t_max;
	Vector3 parabola_initial_velocity;
	Vector3 parabola_accel;
	Vector3 get_position(float p_time, float p_peak_height_offset) {
		return start + parabola_initial_velocity * p_time + parabola_accel * p_time * p_time / 2.0f;
	}
};
static AgentParkourAutojumpData find_autojump_grounded(HBAgent *p_agent, Vector3 p_direction, HBDebugGeometry *debug_geo = nullptr) {
	Ref<Shape3D> coll_shape = p_agent->get_collision_shape();
	const int AUTOJUMP_ITERS_MAX = 15;

	PhysicsDirectSpaceState3D *dss = p_agent->get_world_3d()->get_direct_space_state();

	bool got_gap = false;
	bool got_result = false;
	PhysicsDirectSpaceState3D::RayResult last_hit;

	const float MAX_GAP_DISTANCE = p_agent->get_radius() + 0.1f;

	AgentParkourAutojumpData jump_data;
	for (int i = 0; i < AUTOJUMP_ITERS_MAX; i++) {
		PhysicsDirectSpaceState3D::RayParameters params;
		PhysicsDirectSpaceState3D::RayResult result;

		Vector3 base_pos = p_agent->get_global_position() + p_direction * 3.0f * (i / (float)(AUTOJUMP_ITERS_MAX - 1));
		if (p_agent->get_global_position().distance_to(base_pos) > MAX_GAP_DISTANCE && !got_gap) {
			return jump_data;
		}
		Vector3 target_pos = base_pos;
		base_pos.y += p_agent->get_height() * 0.5f;
		target_pos.y -= p_agent->get_height() * 1.0f;
		params.from = base_pos;
		params.to = target_pos;
		params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
		if (debug_geo) {
			debug_geo->debug_raycast(params);
		}
		if (!dss->intersect_ray(params, result)) {
			got_gap = true;
		} else if (got_gap) {
			if (!is_floor(result.normal, p_agent->get_floor_max_angle())) {
				continue;
			}
			// TODO check if player fits
			got_result = true;
			last_hit = result;
			break;
		}
	}

	if (!got_result) {
		return jump_data;
	}

	jump_data.start = p_agent->get_global_position();
	jump_data.end = last_hit.position;

	Vector3 delta_p = jump_data.end - jump_data.start;
	Vector3 gravity_accel = p_agent->get_agent_constants()->get_gravity();
	const float max_vel = 5.0f;
	float b1 = delta_p.dot(gravity_accel) + max_vel * max_vel;
	float discriminant = b1 * b1 - gravity_accel.length_squared() * delta_p.length_squared();

	if (discriminant < 0.0f) {
		return jump_data;
	}

	float t_low_energy = Math::sqrt(Math::sqrt(4.0f * delta_p.length_squared() / gravity_accel.length_squared()));
	float t_min = Math::sqrt((b1 - Math::sqrt(discriminant)) * 2.0f / gravity_accel.length_squared());
	//t_low_energy = t_min;
	jump_data.found = got_result;
	jump_data.parabola_t_max = t_low_energy;
	jump_data.parabola_initial_velocity = delta_p / t_low_energy - gravity_accel * t_low_energy / 2.0f;
	jump_data.parabola_accel = gravity_accel;

	print_line(jump_data.parabola_initial_velocity.length());

	return jump_data;
}
}; //namespace ParkourAutojump

#endif // AGENT_PARKOUR_H
