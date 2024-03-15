/**************************************************************************/
/*  epas_ik_node.cpp                                                      */
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

#include "epas_ik_node.h"
#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/object/class_db.h"
#include "core/object/object.h"
#include "epas_node.h"
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#endif

void EPASIKNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_ik_influence"), &EPASIKNode::get_ik_influence);
	ClassDB::bind_method(D_METHOD("set_ik_influence", "ik_influence"), &EPASIKNode::set_ik_influence);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ik_influence"), "set_ik_influence", "get_ik_influence");

	ClassDB::bind_method(D_METHOD("get_use_magnet"), &EPASIKNode::get_use_magnet);
	ClassDB::bind_method(D_METHOD("set_use_magnet", "use_magnet"), &EPASIKNode::set_use_magnet);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_magnet"), "set_use_magnet", "get_use_magnet");

	ClassDB::bind_method(D_METHOD("get_target_transform"), &EPASIKNode::get_target_transform);
	ClassDB::bind_method(D_METHOD("set_target_transform", "target_transform"), &EPASIKNode::set_target_transform);
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "target_transform"), "set_target_transform", "get_target_transform");

	ClassDB::bind_method(D_METHOD("get_magnet_position"), &EPASIKNode::get_magnet_position);
	ClassDB::bind_method(D_METHOD("set_magnet_position", "magnet_position"), &EPASIKNode::set_magnet_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "magnet_position"), "set_magnet_position", "get_magnet_position");

	ClassDB::bind_method(D_METHOD("get_ik_end"), &EPASIKNode::get_ik_end);
	ClassDB::bind_method(D_METHOD("set_ik_end", "ik_end"), &EPASIKNode::set_ik_end);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "ik_end"), "set_ik_end", "get_ik_end");
}

#ifdef DEBUG_ENABLED
void EPASIKNode::_debug_node_draw() const {
	ImGui::SetNextItemWidth(50.0f);
	ImGui::SliderFloat("IK Influence", &const_cast<EPASIKNode *>(this)->ik_influence, 0.0f, 1.0f);
}
#endif

void EPASIKNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	Skeleton3D *skel = get_skeleton();
	process_input_pose(0, p_base_pose, p_target_pose, p_delta);

	if (!skel || ik_influence == 0.0f) {
		return;
	}

	int c_bone_idx = skel->find_bone(ik_end);
	int b_bone_idx = skel->get_bone_parent(c_bone_idx);
	int a_bone_idx = skel->get_bone_parent(b_bone_idx);

	if (c_bone_idx == -1 || b_bone_idx == -1 || a_bone_idx == -1) {
		return;
	}

	StringName a_bone_name = skel->get_bone_name(a_bone_idx);
	StringName b_bone_name = skel->get_bone_name(b_bone_idx);
	StringName c_bone_name = ik_end;

	Transform3D a_global_trf = p_target_pose->calculate_bone_global_transform(a_bone_name, skel, p_base_pose);
	Transform3D b_local_trf = p_target_pose->get_bone_transform(b_bone_name, p_base_pose);
	Transform3D c_local_trf = p_target_pose->get_bone_transform(c_bone_name, p_base_pose);

	fabrik_solver->set_joint_transform(0, a_global_trf);
	fabrik_solver->set_joint_transform(1, b_local_trf);
	fabrik_solver->set_joint_transform(2, c_local_trf);
	Transform3D skel_trf = get_skeleton()->get_global_transform();
	fabrik_solver->set_target_position(skel_trf.affine_inverse().xform(target_transform.origin));
	fabrik_solver->set_pole_position(skel->to_local(magnet_position));

	if (!p_target_pose->has_bone(a_bone_name)) {
		p_target_pose->create_bone(a_bone_name);
	}
	if (!p_target_pose->has_bone(b_bone_name)) {
		p_target_pose->create_bone(b_bone_name);
	}
	if (!p_target_pose->has_bone(c_bone_name)) {
		p_target_pose->create_bone(c_bone_name);
	}

	if (length_dirty) {
		length_dirty = false;
		fabrik_solver->calculate_distances();
	}

	fabrik_solver->solve(20);

	Quaternion prev_a_local_rot = p_target_pose->get_bone_rotation(a_bone_name, p_base_pose);
	Quaternion prev_b_local_rot = p_target_pose->get_bone_rotation(b_bone_name, p_base_pose);

	int a_parent_idx = skel->get_bone_parent(a_bone_idx);

	Transform3D a_bone_out_rot = fabrik_solver->get_joint_transform(0);
	Transform3D b_bone_out_rot = fabrik_solver->get_joint_transform(1);

	Quaternion a_parent_global_rot = p_target_pose->calculate_bone_global_transform(skel->get_bone_name(a_parent_idx), skel, p_base_pose).basis.get_rotation_quaternion();
	Quaternion a_final_global_rot = prev_a_local_rot.slerp(a_bone_out_rot.basis.get_rotation_quaternion(), ik_influence);

	Quaternion a_final_rot = prev_a_local_rot.slerp(a_parent_global_rot.inverse() * a_bone_out_rot.basis.get_rotation_quaternion(), ik_influence);
	Quaternion b_final_rot = prev_b_local_rot.slerp(b_bone_out_rot.basis.get_rotation_quaternion(), ik_influence);
	Quaternion b_final_global_rot = a_final_global_rot * b_final_rot;

	Quaternion c_rot = b_final_global_rot.inverse() * p_target_pose->calculate_bone_global_transform(c_bone_name, skel, p_base_pose).basis.get_rotation_quaternion();

	if (use_target_basis) {
		c_rot = skel->get_global_transform().basis.inverse() * target_transform.basis;
		c_rot = b_final_global_rot.inverse() * c_rot;
	}

	p_target_pose->set_bone_rotation(a_bone_name, a_final_rot);
	p_target_pose->set_bone_rotation(b_bone_name, b_final_rot);
	p_target_pose->set_bone_rotation(c_bone_name, c_rot);
}

float EPASIKNode::get_ik_influence() const {
	return ik_influence;
}

void EPASIKNode::set_ik_influence(float p_ik_influence) {
	ik_influence = p_ik_influence;
}

Transform3D EPASIKNode::get_target_transform() const {
	return target_transform;
}

void EPASIKNode::set_target_transform(const Transform3D &p_target_transform) {
	target_transform = p_target_transform;
}

bool EPASIKNode::get_use_magnet() const {
	return use_magnet;
}

void EPASIKNode::set_use_magnet(bool p_use_magnet) {
	use_magnet = p_use_magnet;
	fabrik_solver->set_use_pole_constraint(p_use_magnet);
}

Vector3 EPASIKNode::get_magnet_position() const {
	return magnet_position;
}

void EPASIKNode::set_magnet_position(const Vector3 &p_magnet_position) {
	magnet_position = p_magnet_position;
	fabrik_solver->set_pole_position(p_magnet_position);
}

void EPASIKNode::set_use_hinge(bool p_use_hinge) {
	fabrik_solver->set_joint_hinge_enabled(1, p_use_hinge);
}

bool EPASIKNode::get_use_hinge() const {
	return fabrik_solver->get_joint_hinge_enabled(1);
}

StringName EPASIKNode::get_ik_end() const {
	return ik_end;
}

void EPASIKNode::set_ik_end(const StringName &p_ik_end) {
	ik_end = p_ik_end;
	length_dirty = true;
}

EPASIKNode::EPASIKNode() {
	_set_input_count(1);
	fabrik_solver.instantiate();
}
