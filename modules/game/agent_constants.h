#ifndef AGENT_CONSTANTS_H
#define AGENT_CONSTANTS_H

#include "animation_system/epas_animation.h"
#include "core/io/resource.h"

class HBAgentConstants : public Resource {
	GDCLASS(HBAgentConstants, Resource);

public:
	static constexpr float AGENT_RADIUS = 0.2f;

private:
	float max_move_velocity = 2.8f;
	Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
	float velocity_spring_halflife = 0.175f;
	float tilt_spring_halflife = 0.5f;
	float tilt_max_angle_degrees = 25.0f;

	// General parkour
	float parkour_max_wall_facing_angle_degrees = 60.0f; // Maximum degrees between the desired character movement and the wall's normal to count as facing it
	float parkour_wall_check_distance = 1.25f; // Distance to use for checking for a wall in front of the player
	float parkour_max_ledge_height = 2.0f; // Maximum height a ledge can be directly grabbed from the ground

	// Vault specific
	float vault_max_obstacle_width = 1.0f; // Maximum width of the vaultable obstacle

	float turn_animation_threshold = 45.0f; // If the rotation is bigger than this we initiate a rotation

	// Wall movement
	float ledge_movement_velocity = 0.4f;

	float wallclimb_autojump_distance = 1.5f;

	float default_inertialization_transition_duration = 0.35f;

protected:
	static void _bind_methods();

public:
	enum MovementTransitionInputs {
		MOVEMENT_MOVE = 0,
		MOVEMENT_VAULT = 1,
		MOVEMENT_TURN180_L = 2,
		MOVEMENT_TURN180_R = 3,
		MOVEMENT_WALLRUN = 4,
		MOVEMENT_WALLGRABBED = 5,
		MOVEMENT_FALL = 6,
		MOVEMENT_LEDGE_GETUP = 7,
		MOVEMENT_PARKOUR_CAT = 8,
		MOVEMENT_MOVE_BEAM = 9,
		MOVEMENT_LEDGE_DROP = 10,
		MOVEMENT_LEDGE_TO_GROUND = 11,
		MOVEMENT_LEDGE_HOP_UP = 12,
		MOVEMENT_STANDING_DROP_TO_LEDGE = 13,
		MOVEMENT_STANDING_JUMP_TO_LEDGE = 14,
		MOVEMENT_SHORT_HOP = 15,
		MOVEMENT_BACK_EJECT_TO_LEDGE = 16,
		MOVEMENT_BACK_EJECT_TO_SURFACE = 17,
		MOVEMENT_WALLPARKOUR_UP_LONG_JUMP = 18,
		MOVEMENT_WALLPARKOUR_DOWN_LONG_JUMP = 19,
		MOVEMENT_WALLPARKOUR_LEFT_LONG_JUMP = 20,
		MOVEMENT_WALLPARKOUR_RIGHT_LONG_JUMP = 21,
		MOVEMENT_COMBAT_ATTACK = 22,
		MOVEMENT_COMBAT_HIT_RIGHT = 23,
		MOVEMENT_COMBAT_PARRY = 24,
		MOVEMENT_COMBAT_PARRIED = 25,
		MOVEMENT_COMBAT_ROLL = 26,
		MOVEMENT_MAX
	};

	static const uint64_t PARRY_WINDOW = 500 * 1000; // In microseconds

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

	float get_parkour_max_wall_facing_angle_degrees() const;
	void set_parkour_max_wall_facing_angle_degrees(float p_parkour_max_wall_facing_angle_degrees);

	float get_vault_max_obstacle_width() const;
	void set_vault_max_obstacle_width(float p_vault_max_obstacle_width);

	float get_parkour_wall_check_distance() const;
	void set_parkour_wall_check_distance(float p_parkour_wall_check_distance);

	float get_turn_animation_threshold_degrees() const;
	void set_turn_animation_threshold_degrees(float p_turn_animation_threshold_degrees);

	float get_parkour_max_ledge_height() const;
	void set_parkour_max_ledge_height(float p_parkour_max_ledge_height);

	float get_ledge_movement_velocity() const;
	void set_ledge_movement_velocity(float p_ledge_movement_velocity);

	float get_default_inertialization_transition_duration() const;
	void set_default_inertialization_transition_duration(float p_default_inertialization_transition_duration);
};

VARIANT_ENUM_CAST(HBAgentConstants::MovementTransitionInputs);

#endif // AGENT_CONSTANTS_H
