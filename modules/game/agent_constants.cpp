#include "agent_constants.h"

void HBAgentConstants::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_max_move_velocity"), &HBAgentConstants::get_max_move_velocity);
	ClassDB::bind_method(D_METHOD("set_max_move_velocity", "max_move_velocity"), &HBAgentConstants::set_max_move_velocity);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_move_velocity"), "set_max_move_velocity", "get_max_move_velocity");

	ClassDB::bind_method(D_METHOD("get_gravity"), &HBAgentConstants::get_gravity);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &HBAgentConstants::set_gravity);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");

	ClassDB::bind_method(D_METHOD("get_velocity_spring_halflife"), &HBAgentConstants::get_velocity_spring_halflife);
	ClassDB::bind_method(D_METHOD("set_velocity_spring_halflife", "velocity_spring_halflife"), &HBAgentConstants::set_velocity_spring_halflife);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "velocity_spring_halflife"), "set_velocity_spring_halflife", "get_velocity_spring_halflife");

	ClassDB::bind_method(D_METHOD("get_tilt_spring_halflife"), &HBAgentConstants::get_tilt_spring_halflife);
	ClassDB::bind_method(D_METHOD("set_tilt_spring_halflife", "tilt_spring_halflife"), &HBAgentConstants::set_tilt_spring_halflife);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tilt_spring_halflife"), "set_tilt_spring_halflife", "get_tilt_spring_halflife");

	ClassDB::bind_method(D_METHOD("get_tilt_max_angle_degrees"), &HBAgentConstants::get_tilt_max_angle_degrees);
	ClassDB::bind_method(D_METHOD("set_tilt_max_angle_degrees", "tilt_max_angle_degrees"), &HBAgentConstants::set_tilt_max_angle_degrees);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tilt_max_angle_degrees", PROPERTY_HINT_RANGE, "0,180,degrees"), "set_tilt_max_angle_degrees", "get_tilt_max_angle_degrees");

	ClassDB::bind_method(D_METHOD("get_parkour_max_wall_facing_angle_degrees"), &HBAgentConstants::get_parkour_max_wall_facing_angle_degrees);
	ClassDB::bind_method(D_METHOD("set_parkour_max_wall_facing_angle_degrees", "parkour_max_wall_facing_angle_degrees"), &HBAgentConstants::set_parkour_max_wall_facing_angle_degrees);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "parkour_max_wall_facing_angle_degrees"), "set_parkour_max_wall_facing_angle_degrees", "get_parkour_max_wall_facing_angle_degrees");

	ClassDB::bind_method(D_METHOD("get_vault_max_obstacle_width"), &HBAgentConstants::get_vault_max_obstacle_width);
	ClassDB::bind_method(D_METHOD("set_vault_max_obstacle_width", "vault_max_obstacle_width"), &HBAgentConstants::set_vault_max_obstacle_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vault_max_obstacle_width"), "set_vault_max_obstacle_width", "get_vault_max_obstacle_width");

	ClassDB::bind_method(D_METHOD("get_turn_animation_threshold"), &HBAgentConstants::get_turn_animation_threshold_degrees);
	ClassDB::bind_method(D_METHOD("set_turn_animation_threshold", "turn_animation_threshold"), &HBAgentConstants::set_turn_animation_threshold_degrees);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "turn_animation_threshold"), "set_turn_animation_threshold", "get_turn_animation_threshold");

	ClassDB::bind_method(D_METHOD("get_ledge_movement_velocity"), &HBAgentConstants::get_ledge_movement_velocity);
	ClassDB::bind_method(D_METHOD("set_ledge_movement_velocity", "ledge_movement_velocity"), &HBAgentConstants::set_ledge_movement_velocity);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ledge_movement_velocity"), "set_ledge_movement_velocity", "get_ledge_movement_velocity");

	ClassDB::bind_method(D_METHOD("get_default_inertialization_transition_duration"), &HBAgentConstants::get_default_inertialization_transition_duration);
	ClassDB::bind_method(D_METHOD("set_default_inertialization_transition_duration", "default_inertialization_transition_duration"), &HBAgentConstants::set_default_inertialization_transition_duration);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "default_inertialization_transition_duration"), "set_default_inertialization_transition_duration", "get_default_inertialization_transition_duration");
	BIND_ENUM_CONSTANT(MOVEMENT_MOVE);
	BIND_ENUM_CONSTANT(MOVEMENT_VAULT);
	BIND_ENUM_CONSTANT(MOVEMENT_TURN180_L);
	BIND_ENUM_CONSTANT(MOVEMENT_TURN180_R);
	BIND_ENUM_CONSTANT(MOVEMENT_WALLRUN);
	BIND_ENUM_CONSTANT(MOVEMENT_WALLGRABBED);
	BIND_ENUM_CONSTANT(MOVEMENT_FALL);
	BIND_ENUM_CONSTANT(MOVEMENT_LEDGE_GETUP);
	BIND_ENUM_CONSTANT(MOVEMENT_PARKOUR_CAT);
	BIND_ENUM_CONSTANT(MOVEMENT_MOVE_BEAM);
	BIND_ENUM_CONSTANT(MOVEMENT_LEDGE_DROP);
	BIND_ENUM_CONSTANT(MOVEMENT_LEDGE_TO_GROUND);
	BIND_ENUM_CONSTANT(MOVEMENT_LEDGE_HOP_UP);
	BIND_ENUM_CONSTANT(MOVEMENT_STANDING_DROP_TO_LEDGE);
	BIND_ENUM_CONSTANT(MOVEMENT_STANDING_JUMP_TO_LEDGE);
	BIND_ENUM_CONSTANT(MOVEMENT_SHORT_HOP);
	BIND_ENUM_CONSTANT(MOVEMENT_BACK_EJECT_TO_LEDGE);
	BIND_ENUM_CONSTANT(MOVEMENT_BACK_EJECT_TO_SURFACE);
	BIND_ENUM_CONSTANT(MOVEMENT_WALLPARKOUR_UP_LONG_JUMP);
	BIND_ENUM_CONSTANT(MOVEMENT_WALLPARKOUR_DOWN_LONG_JUMP);
	BIND_ENUM_CONSTANT(MOVEMENT_WALLPARKOUR_LEFT_LONG_JUMP);
	BIND_ENUM_CONSTANT(MOVEMENT_WALLPARKOUR_RIGHT_LONG_JUMP);
	BIND_ENUM_CONSTANT(MOVEMENT_COMBAT_ATTACK);
	BIND_ENUM_CONSTANT(MOVEMENT_COMBAT_HIT_RIGHT);
	BIND_ENUM_CONSTANT(MOVEMENT_COMBAT_PARRY);
	BIND_ENUM_CONSTANT(MOVEMENT_COMBAT_ROLL);
	BIND_ENUM_CONSTANT(MOVEMENT_MAX);
}

float HBAgentConstants::get_max_move_velocity() const {
	return max_move_velocity;
}

void HBAgentConstants::set_max_move_velocity(float p_max_move_velocity) {
	max_move_velocity = p_max_move_velocity;
}

void HBAgentConstants::set_gravity(const Vector3 &p_gravity) {
	gravity = p_gravity;
}

Vector3 HBAgentConstants::get_gravity() const {
	return gravity;
}

float HBAgentConstants::get_velocity_spring_halflife() const {
	return velocity_spring_halflife;
}

void HBAgentConstants::set_velocity_spring_halflife(float p_velocity_spring_halflife) {
	velocity_spring_halflife = p_velocity_spring_halflife;
}

float HBAgentConstants::get_tilt_spring_halflife() const {
	return tilt_spring_halflife;
}

void HBAgentConstants::set_tilt_spring_halflife(float p_tilt_spring_halflife) {
	tilt_spring_halflife = p_tilt_spring_halflife;
}

float HBAgentConstants::get_tilt_max_angle_degrees() const {
	return tilt_max_angle_degrees;
}

void HBAgentConstants::set_tilt_max_angle_degrees(float p_tilt_max_angle_degrees) {
	tilt_max_angle_degrees = p_tilt_max_angle_degrees;
}

float HBAgentConstants::get_parkour_max_wall_facing_angle_degrees() const {
	return parkour_max_wall_facing_angle_degrees;
}

void HBAgentConstants::set_parkour_max_wall_facing_angle_degrees(float p_parkour_max_wall_facing_angle_degrees) {
	parkour_max_wall_facing_angle_degrees = p_parkour_max_wall_facing_angle_degrees;
}

float HBAgentConstants::get_vault_max_obstacle_width() const {
	return vault_max_obstacle_width;
}

void HBAgentConstants::set_vault_max_obstacle_width(float p_vault_max_obstacle_width) {
	vault_max_obstacle_width = p_vault_max_obstacle_width;
}

float HBAgentConstants::get_parkour_wall_check_distance() const {
	return parkour_wall_check_distance;
}

void HBAgentConstants::set_parkour_wall_check_distance(float p_vault_check_distance) {
	parkour_wall_check_distance = p_vault_check_distance;
}

float HBAgentConstants::get_turn_animation_threshold_degrees() const {
	return turn_animation_threshold;
}

void HBAgentConstants::set_turn_animation_threshold_degrees(float p_turn_animation_threshold) {
	turn_animation_threshold = p_turn_animation_threshold;
}

float HBAgentConstants::get_parkour_max_ledge_height() const {
	return parkour_max_ledge_height;
}

void HBAgentConstants::set_parkour_max_ledge_height(float p_parkour_max_ledge_height) {
	parkour_max_ledge_height = p_parkour_max_ledge_height;
}

float HBAgentConstants::get_ledge_movement_velocity() const {
	return ledge_movement_velocity;
}

void HBAgentConstants::set_ledge_movement_velocity(float p_ledge_movement_velocity) {
	ledge_movement_velocity = p_ledge_movement_velocity;
}

float HBAgentConstants::get_default_inertialization_transition_duration() const {
	return default_inertialization_transition_duration;
}

void HBAgentConstants::set_default_inertialization_transition_duration(float p_default_inertialization_transition_duration) {
	default_inertialization_transition_duration = p_default_inertialization_transition_duration;
}
