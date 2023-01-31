
#ifndef ACTOR_H
#define ACTOR_H

#include "scene/3d/physics_body_3d.h"

class HBActor : public CharacterBody3D {
	GDCLASS(HBActor, CharacterBody3D);

public:
	enum MovementMode {
		MOVE_GROUNDED
	};

private:
	MovementMode movement_mode = MovementMode::MOVE_GROUNDED;

	Vector3 gravity = Vector3(0.0f, -20.81f, 0.0f);
	float move_velocity = 7.0f;
	float velocity_spring_halflife = 0.175f;
	Vector3 velocity_spring_acceleration;

	Vector3 tilt_spring_velocity;
	float tilt_spring_halflife = 0.3f;

	NodePath graphics_node;
	NodePath tilt_node;
	ObjectID tilt_node_cache;
	Vector3 graphics_lookat_normal;
	ObjectID graphics_node_cache;

	void _update_graphics_node_cache();
	void _update_tilt_node_cache();
	Node3D *_get_graphics_node();
	Node3D *_get_tilt_node();
	void _rotate_towards_velocity(float p_delta);
	void _tilt_towards_acceleration(float p_delta);

	void _physics_process(float p_delta);

protected:
	static void _bind_methods();
	void _notification(int p_what);
	virtual Vector3 get_input() const { return Vector3(); };

public:
	void set_graphics_node(NodePath p_path);
	NodePath get_graphics_node() const;
	void set_tilt_node(NodePath p_path);
	NodePath get_tilt_node() const;
	void set_movement_mode(MovementMode p_movement_mode);
	MovementMode get_movement_mode() const;
	HBActor();
	virtual ~HBActor();
};

#endif // ACTOR_H
