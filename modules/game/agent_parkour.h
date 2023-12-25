#ifndef AGENT_PARKOUR_H
#define AGENT_PARKOUR_H

#include "agent.h"
#include "debug_geometry.h"
#include "modules/tbloader/src/tb_loader_singleton.h"
#include "physics_layers.h"
#include "scene/3d/area_3d.h"
#include "scene/3d/collision_shape_3d.h"
#include "scene/3d/path_3d.h"
#include "scene/3d/physics_body_3d.h"
#include "scene/resources/box_shape_3d.h"
#include "scene/resources/sphere_shape_3d.h"

class HBAgentParkourLedge : public Area3D {
	GDCLASS(HBAgentParkourLedge, Area3D);
	Ref<Curve3D> curve;
	// Curve the agent will follow
	Ref<Curve3D> agent_curve;

protected:
	static void _bind_methods();

public:
	Ref<Curve3D> get_curve() const;
	void set_curve(const Ref<Curve3D> &p_curve);
	void generate_colliders();
	float get_closest_offset(const Vector3 &p_global_pos) const;
	float get_closest_offset_agent(const Vector3 &p_global_pos) const;
	Transform3D get_ledge_transform_at_offset(float p_offset) const;
	Transform3D get_agent_ledge_transform_at_offset(float p_offset) const;
	bool check_agent_fits(HBAgent *p_agent, float p_offset, HBDebugGeometry *p_debug_geo = nullptr) const;

	HBAgentParkourLedge();

	Ref<Curve3D> get_agent_curve() const { return agent_curve; }
	void set_agent_curve(const Ref<Curve3D> &agent_curve_) { agent_curve = agent_curve_; }
};

class HBAgentParkourPoint : public StaticBody3D, public TBLoaderEntity {
	GDCLASS(HBAgentParkourPoint, StaticBody3D);
	bool collision_shape_dirty = true;
	Ref<SphereShape3D> shape;
	CollisionShape3D *collision_shape = nullptr;
	void _update_collision_shape();

protected:
	void _notification(int p_what);

public:
	HBAgentParkourPoint();
	static StringName get_entity_name() {
		return "func_parkour_point";
	}
};

class HBAgentParkourBeam : public Area3D, public TBLoaderEntity {
	GDCLASS(HBAgentParkourBeam, Area3D);
	Ref<Curve3D> curve;

	MeshInstance3D *debug_preview = nullptr;
	bool line_redraw_queued = false;
	void _line_redraw();

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	virtual void _editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) override;
	void set_curve(Ref<Curve3D> p_curve);
	Ref<Curve3D> get_curve() const;
	void _queue_line_redraw();
	static StringName get_entity_name() {
		return "func_parkour_beam";
	}

	HBAgentParkourBeam();
};

namespace ParkourAutojump {
static bool is_wall(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) > p_floor_max_angle;
}

static bool is_floor(const Vector3 &p_normal, float p_floor_max_angle) {
	return p_normal.angle_to(Vector3(0.0f, 1.0f, 0.0f)) <= p_floor_max_angle;
}

static void calculate_target_as_apex(const Vector3 &p_origin, const Vector3 &p_target, const float &p_gravity, Vector3 &p_force, float &p_time) {
	float v0y = Math::sqrt(2.0f * p_gravity * (p_target.y - p_origin.y));
	p_time = v0y / p_gravity;

	Vector3 v0z = p_target - p_origin;
	v0z.y = 0.0f;
	v0z /= p_time;
	p_force = v0z;
	p_force.y = v0y;
}

static void calculate_apex_force(const Vector3 &p_origin, const Vector3 &p_target, const float &p_gravity, const float &p_apex_height, Vector3 &p_force, float &p_time) {
	float apx = p_apex_height;
	apx = CLAMP(apx + (p_target.y - p_origin.y), 0.0f, apx);
	float v0y = Math::sqrt(2.0 * p_gravity * apx);
	float a = p_gravity * 0.5f;
	float b = -v0y;
	float c = p_target.y - p_origin.y;
	float d = b * b - 4.0 * a * c;

	if (d < 0.0) {
		return calculate_target_as_apex(p_origin, p_target, p_gravity, p_force, p_time);
	}

	float sqrt_d = Math::sqrt(d);
	float x1 = (-b + sqrt_d) / (2.0 * a);
	float x2 = (-b - sqrt_d) / (2.0 * a);
	p_time = MAX(x1, x2);

	Vector3 v0z = p_target - p_origin;
	v0z.y = 0.0f;
	v0z /= p_time;

	p_force = v0z;
	p_force.y = v0y;
}

static Vector3 calculate_position_at(const Vector3 &p_start_pos, const Vector3 &p_force, const float p_gravity, const float p_time) {
	float x = p_start_pos.x + p_force.x * p_time;
	float z = p_start_pos.z + p_force.z * p_time;
	float y = p_start_pos.y + p_force.y * p_time - 0.5f * p_gravity * p_time * p_time;

	return Vector3(x, y, z);
}

// Generic autojump solver
struct AgentParkourAutojumpData {
	bool found = false;
	Vector3 start;
	Vector3 end;
	Vector3 end_normal;
	float parabola_t_max;
	Vector3 parabola_force;
	float parabola_gravity;
	Vector3 get_position(float p_time) {
		return calculate_position_at(start, parabola_force, parabola_gravity, p_time);
	}
};

struct AutojumpSettings {
	bool needs_gap = false;
	float max_gap_distance = 1.0f;
	int iterations = 32;
	float max_jump_distance = 3.0f;
	float ray_down_offset = -0.5f;
	float apex_height = 0.5f;

	AgentParkourAutojumpData find_autojump(HBAgent *p_agent, Vector3 p_direction, HBDebugGeometry *p_debug_geo = nullptr) {
		Ref<Shape3D> coll_shape = p_agent->get_collision_shape();

		PhysicsDirectSpaceState3D *dss = p_agent->get_world_3d()->get_direct_space_state();

		bool got_gap = false;
		bool got_result = false;
		PhysicsDirectSpaceState3D::RayResult last_hit;

		AgentParkourAutojumpData jump_data;
		Vector3 gravity_accel = p_agent->get_agent_constants()->get_gravity();

		if (p_debug_geo) {
			p_debug_geo->debug_line(p_agent->get_position(), p_agent->get_position() + p_direction, Color("RED"));
		}
		Ref<Shape3D> shape = p_agent->get_collision_shape();

		for (int i = 1; i < iterations; i++) {
			PhysicsDirectSpaceState3D::RayParameters params;
			PhysicsDirectSpaceState3D::RayResult result;
			params.hit_back_faces = false;

			Vector3 base_pos = p_agent->get_global_position() + p_direction * max_jump_distance * (i / (float)(iterations - 1));
			if (p_agent->get_global_position().distance_to(base_pos) > (p_agent->get_radius() + max_gap_distance) && (needs_gap && !got_gap)) {
				//return jump_data;
			}
			Vector3 target_pos = base_pos;
			base_pos.y += p_agent->get_height() * 0.5f;
			target_pos.y += ray_down_offset;
			params.from = base_pos;
			params.to = target_pos;
			params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
			if (p_debug_geo) {
				p_debug_geo->debug_raycast(params);
			}
			if (!dss->intersect_ray(params, result)) {
				got_gap = true;
				continue;
			}

			if ((needs_gap && !got_gap)) {
				continue;
			}

			if (p_debug_geo) {
				p_debug_geo->debug_sphere(result.position);
			}
			if (!is_floor(result.normal, p_agent->get_floor_max_angle())) {
				continue;
			}
			// TODO check if player fits
			got_result = true;
			last_hit = result;

			if (needs_gap && got_gap) {
				break;
			}
		}

		if (!got_result) {
			return jump_data;
		}

		jump_data.start = p_agent->get_global_position();
		jump_data.end = last_hit.position;

		jump_data.found = got_result;
		jump_data.parabola_gravity = -gravity_accel.y;
		calculate_apex_force(jump_data.start, jump_data.end, -gravity_accel.y, apex_height, jump_data.parabola_force, jump_data.parabola_t_max);

		return jump_data;
	}

	AgentParkourAutojumpData find_autojump_from_to(HBAgent *p_agent, Vector3 p_from, Vector3 p_to) {
		AgentParkourAutojumpData jump_data;
		Vector3 gravity_accel = p_agent->get_agent_constants()->get_gravity() * 1.0;
		jump_data.found = true;
		jump_data.parabola_gravity = -gravity_accel.y;
		jump_data.start = p_from;
		jump_data.end = p_to;
		calculate_apex_force(jump_data.start, jump_data.end, -gravity_accel.y, apex_height, jump_data.parabola_force, jump_data.parabola_t_max);

		return jump_data;
	}
};
}; //namespace ParkourAutojump

#endif // AGENT_PARKOUR_H
