/**************************************************************************/
/*  epas_lookat_node.cpp                                                  */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "epas_lookat_node.h"
#include "../springs.h"

void EPASLookatNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_skeleton_forward", "skeleton_forward"), &EPASLookatNode::set_skeleton_forward);
	ClassDB::bind_method(D_METHOD("get_skeleton_forward"), &EPASLookatNode::get_skeleton_forward);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "skeleton_forward"), "set_skeleton_forward", "get_skeleton_forward");

	ClassDB::bind_method(D_METHOD("set_target_world", "target_world"), &EPASLookatNode::set_target_world);
	ClassDB::bind_method(D_METHOD("get_target_world"), &EPASLookatNode::get_target_world);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target_world"), "set_target_world", "get_target_world");

	ClassDB::bind_method(D_METHOD("set_bone_name", "bone_name"), &EPASLookatNode::set_bone_name);
	ClassDB::bind_method(D_METHOD("get_bone_name"), &EPASLookatNode::get_bone_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "bone_name"), "set_bone_name", "get_bone_name");

	ClassDB::bind_method(D_METHOD("set_spring_halflife", "spring_halflife"), &EPASLookatNode::set_spring_halflife);
	ClassDB::bind_method(D_METHOD("get_spring_halflife"), &EPASLookatNode::get_spring_halflife);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_halflife"), "set_spring_halflife", "get_spring_halflife");

	ClassDB::bind_method(D_METHOD("set_max_angle_degrees", "max_angle_degrees"), &EPASLookatNode::set_max_angle_degrees);
	ClassDB::bind_method(D_METHOD("get_max_angle_degrees"), &EPASLookatNode::get_max_angle_degrees);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_angle_degrees"), "set_max_angle_degrees", "get_max_angle_degrees");
}

Vector3 EPASLookatNode::get_skeleton_forward() const {
	return skeleton_forward;
}
void EPASLookatNode::set_skeleton_forward(const Vector3 &p_skeleton_forward) {
	skeleton_forward = p_skeleton_forward;
}

void EPASLookatNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	process_input_pose(0, p_base_pose, p_target_pose, p_delta);

	if (influence == 0.0f) {
		return;
	}

	Transform3D base_trf = p_base_pose->calculate_bone_global_transform(bone_name, get_skeleton());
	Vector3 bone_local_forward = base_trf.basis.xform_inv(skeleton_forward).normalized();
	Vector3 bone_local_up = base_trf.basis.xform_inv(skeleton_rotation_axis).normalized();

	Transform3D current_bone_trf = p_target_pose->calculate_bone_global_transform(bone_name, get_skeleton(), p_base_pose);
	Vector3 current_skeleton_forward = current_bone_trf.basis.xform(bone_local_forward).normalized();

	Vector3 skel_trg = get_skeleton()->get_global_transform().xform_inv(target_world);
	Vector3 trg_skeleton_forward = current_bone_trf.origin.direction_to(skel_trg);

	Quaternion trg_skel_rot_offset = Quaternion(current_skeleton_forward, trg_skeleton_forward);

	Quaternion target_rot = trg_skel_rot_offset * current_bone_trf.basis;
	target_rot = Quaternion(target_rot.xform(bone_local_up), skeleton_rotation_axis) * target_rot;
	target_rot.normalize();

	float angle = current_bone_trf.basis.get_rotation_quaternion().angle_to(target_rot);
	float angle_amount = MIN(angle, Math::deg_to_rad((float)max_angle_degrees)) / angle;
	target_rot = current_bone_trf.basis.get_rotation_quaternion().slerp(target_rot, angle_amount);

	if (!has_target_rotation) {
		quaternion_spring_velocity = Vector3();
		quaternion_spring_value = current_bone_trf.basis.get_rotation_quaternion();
		has_target_rotation = true;
	}

	HBSprings::simple_spring_damper_exact_quat(quaternion_spring_value, quaternion_spring_velocity, target_rot, spring_halflife, p_delta);

	Transform3D final_bone_trg = current_bone_trf;

	final_bone_trg.basis = final_bone_trg.basis.get_rotation_quaternion().slerp(quaternion_spring_value, influence);

	if (!p_target_pose->has_bone(bone_name)) {
		p_target_pose->create_bone(bone_name);
	}

	StringName parent_name = get_skeleton()->get_bone_name(get_skeleton()->get_bone_parent(get_skeleton()->find_bone(bone_name)));
	Transform3D parent_trf = p_target_pose->calculate_bone_global_transform(parent_name, get_skeleton(), p_base_pose);

	final_bone_trg = parent_trf.affine_inverse() * final_bone_trg;

	p_target_pose->set_bone_rotation(bone_name, final_bone_trg.basis.get_rotation_quaternion());
}

EPASLookatNode::EPASLookatNode() {
	_set_input_count(1);
}

Vector3 EPASLookatNode::get_skeleton_rotation_axis() const { return skeleton_rotation_axis; }

void EPASLookatNode::set_skeleton_rotation_axis(const Vector3 &p_skeleton_rotation_axis) { skeleton_rotation_axis = p_skeleton_rotation_axis; }

Vector3 EPASLookatNode::get_target_world() const { return target_world; }

void EPASLookatNode::set_target_world(const Vector3 &p_target_world) { target_world = p_target_world; }

StringName EPASLookatNode::get_bone_name() const { return bone_name; }

void EPASLookatNode::set_bone_name(const StringName &p_bone_name) { bone_name = p_bone_name; }

float EPASLookatNode::get_influence() const { return influence; }

void EPASLookatNode::set_influence(float p_influence) { influence = p_influence; }

float EPASLookatNode::get_spring_halflife() const { return spring_halflife; }

void EPASLookatNode::set_spring_halflife(float p_spring_halflife) { spring_halflife = p_spring_halflife; }

void EPASLookatNode::reset() {
	has_target_rotation = false;
}

#ifdef DEBUG_ENABLED

#include "modules/imgui/godot_imgui.h"

void EPASLookatNode::_debug_node_draw() const {
	//ImGui::SliderFloat("Spring HalfLife", &(const_cast<EPASLookatNode *>(this)->spring_halflife), 0.01f, 4.0f);
}

#endif

float EPASLookatNode::get_max_angle_degrees() const { return max_angle_degrees; }

void EPASLookatNode::set_max_angle_degrees(float p_max_angle_degrees) { max_angle_degrees = p_max_angle_degrees; }
