#ifndef AGENT_CONSTANTS_H
#define AGENT_CONSTANTS_H

#include "core/io/resource.h"

class HBAgentConstants : public Resource {
	GDCLASS(HBAgentConstants, Resource);

	float max_move_velocity = 2.8f;
	Vector3 gravity = Vector3(0.0f, -20.81f, 0.0f);
	float velocity_spring_halflife = 0.175f;
	float tilt_spring_halflife = 0.5f;
	float tilt_max_angle_degrees = 25.0f;

protected:
	static void _bind_methods();

public:
	float get_max_move_velocity() const;
	void set_max_move_velocity(float p_max_move_velocity);

	Vector3 get_gravity() const;
	void set_gravity(const Vector3 &p_gravity);

	float get_velocity_spring_halflife() const;
	void set_velocity_spring_halflife(float p_velocity_spring_halflife);

	float get_tilt_spring_halflife() const;
	void set_tilt_spring_halflife(float p_tilt_spring_halflife);

	float get_tilt_max_angle_degrees() const;
	void set_tilt_max_angle_degrees(float p_tilt_max_angle_degrees);
};

#endif // AGENT_CONSTANTS_H
