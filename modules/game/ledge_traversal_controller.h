#ifndef LEDGE_TRAVERSAL_CONTROLLER_H
#define LEDGE_TRAVERSAL_CONTROLLER_H

#include "agent_procedural_anim.h"
#include "debug_geometry.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/physics_body_3d.h"

class HBLedgeTraversalController : public CharacterBody3D {
	GDCLASS(HBLedgeTraversalController, CharacterBody3D);
	Ref<Shape3D> limb_test_shape;
	Transform3D limb_transforms[AgentProceduralAnimator::LIMB_MAX];
	bool limb_dangle_status[AgentProceduralAnimator::LIMB_MAX] = { false };
	float radius = 0.25f;
	float height = 1.35f;
	float max_movement_velocity = 0.4f;
	float movement_velocity_target = 0.0f;
	float movement_velocity = 0.0f;
	float movement_acceleration = 0.4f;
	Quaternion out_rotation;
	Transform3D turn_in_place_values[2];
	bool is_turning_in_place = false;
	Transform3D wall_being_rotated_towards;
	float turn_progress = 0.0f;
	int rotation_movement_dir = 0;
	Transform3D current_wall;

	Vector3 movement_input;
	Vector3 get_transformed_movement_input() const;
	void _handle_collision(const Vector3 &p_previous_pos);
	MeshInstance3D *movement_dir_debug_graphic = nullptr;
	void _update_movement_dir_debug_graphic();
	HBDebugGeometry *debug_geo = nullptr;
	void _handle_turn_in_place(float p_delta);
	void _handle_limbs();

	CharacterBody3D *limb_test_body = nullptr;

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void move_to_ledge(const Transform3D &p_ledge_transform);
	void update(float p_movement_input);

	Vector3 get_movement_input() const;
	void set_movement_input(const Vector3 &p_movement_input);
	Quaternion get_graphics_rotation() const;
	bool get_limb_is_dangling(AgentProceduralAnimator::AgentLimb p_limb) const;
	Transform3D get_limb_transform(AgentProceduralAnimator::AgentLimb p_limb) const;
	float get_movement_velocity() const;
	float get_max_movement_velocity() const { return max_movement_velocity; };
	void set_max_movement_velocity(float p_max_movement_velocity) { max_movement_velocity = p_max_movement_velocity; };
};

#endif // LEDGE_TRAVERSAL_CONTROLLER_H
