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
