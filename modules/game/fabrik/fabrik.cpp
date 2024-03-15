/**************************************************************************/
/*  fabrik.cpp                                                            */
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

#include "fabrik.h"
#include "modules/tracy/tracy.gen.h"
#include "scene/3d/mesh_instance_3d.h"

void FABRIKSolver::_solve_backwards() {
	joints.write[joints.size() - 1].working_position = target_position;
	for (int i = joints.size() - 2; i >= 0; i--) {
		float distance = joints[i + 1].distance_to_parent;
		Vector3 dir = joints[i + 1].working_position.direction_to(joints[i].working_position);
		joints.write[i].working_position = joints[i + 1].working_position + dir * distance;
	}
}

void FABRIKSolver::_solve_forwards(const Vector3 &p_base) {
	joints.write[0].working_position = p_base;
	for (int i = 1; i < joints.size(); i++) {
		float distance = joints[i].distance_to_parent;
		Vector3 dir = joints[i - 1].working_position.direction_to(joints[i].working_position);
		joints.write[i].working_position = joints[i - 1].working_position + dir * distance;
	}
}

void FABRIKSolver::_process_pole_vector() {
	if (joints.size() < 3) {
		return;
	}
	// First we aim the chain towards the target vector, in a pseudo-ccd step
	_local_to_global();
	Vector3 start = joints[0].global_transform.origin;
	Vector3 end = joints[joints.size() - 1].global_transform.origin;
	Vector3 curr_dir = (end - start).normalized();
	Vector3 new_dir = (target_position - start).normalized();
	// This is fine because joint 0 is in global space
	joints.write[0].transform.basis = Basis(Quaternion(curr_dir, new_dir) * joints[0].transform.basis.get_rotation_quaternion());

	_local_to_global();
	// Now we aim the chain towards our target
	Vector3 plane_normal = joints[joints.size() - 1].global_transform.origin - joints[0].global_transform.origin;
	if (plane_normal.is_zero_approx()) {
		return;
	}
	plane_normal.normalize();

	Plane plane = Plane(plane_normal, joints[0].global_transform.origin);
	Vector3 pole_proj = plane.project(pole_position);
	int middle = joints.size() / 2;
	Vector3 middle_proj = plane.project(joints[middle].global_transform.origin);
	Vector3 diff_pole = (pole_proj - joints[0].global_transform.origin).normalized();
	Vector3 diff_middle = (middle_proj - joints[0].global_transform.origin).normalized();
	float angle_change = diff_middle.angle_to(diff_pole);
	if (Math::is_zero_approx(angle_change)) {
		return;
	}
	joints.write[0].transform.basis = Quaternion(diff_middle, diff_pole) * joints[0].global_transform.basis.get_rotation_quaternion();
}

Transform3D FABRIKSolver::get_global_trf(int p_idx) {
	Transform3D trf = joints[p_idx].transform;
	for (int i = p_idx - 1; i >= 0; i--) {
		trf = joints[i].transform * trf;
	}
	return trf;
}

void FABRIKSolver::_apply_fabrik() {
	Transform3D prev_global_trf;
	for (int i = 0; i < joints.size() - 1; i++) {
		FABRIKJoint &joint = joints.write[i];
		Transform3D joint_global = prev_global_trf * joint.transform;
		Transform3D joint_global_next = joint_global * joints[i + 1].transform;

		Vector3 pos = joint_global.origin;

		Vector3 from = pos.direction_to(joint_global_next.origin);
		Vector3 to = pos.direction_to(joints[i + 1].working_position);

		joint.global_transform = joint_global;
		joint.global_transform.basis = Basis(Quaternion(from, to) * joint_global.basis.get_rotation_quaternion());

		if (joint.rotation_limit_enabled) {
			WARN_PRINT_ONCE("Using joint rotation limits, this is currently broken, please don't use");
			Transform3D local = prev_global_trf.affine_inverse() * joint.global_transform;
			Vector3 euler = local.basis.get_rotation_quaternion().get_euler();
			euler = euler.clamp(joint.min_rotation, joint.max_rotation);
			local.basis = Basis(Quaternion::from_euler(euler));
			joint.transform = local;
			joint.global_transform = prev_global_trf * local;
		} else {
			joint.transform = prev_global_trf.affine_inverse() * joint.global_transform;
		}

		if (joint.enable_hinge) {
			Vector3 parent_hinge_axis = prev_global_trf.basis.get_rotation_quaternion().xform(joint.hinge_axis);
			Vector3 current_hinge_axis = joint.global_transform.xform(joint.hinge_axis);

			Quaternion hinge_delta = Quaternion(current_hinge_axis, parent_hinge_axis);
			joint.global_transform.basis = Basis(hinge_delta * joint.global_transform.basis.get_rotation_quaternion());
		}

		prev_global_trf = joint.global_transform;
	}
}

void FABRIKSolver::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_joint_count", "joint_count"), &FABRIKSolver::set_joint_count);
	ClassDB::bind_method(D_METHOD("get_joint_count"), &FABRIKSolver::get_joint_count);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "joint_count"), "set_joint_count", "get_joint_count");

	ClassDB::bind_method(D_METHOD("set_joint_transform", "joint_idx", "joint_transform"), &FABRIKSolver::set_joint_transform);
	ClassDB::bind_method(D_METHOD("get_joint_transform", "joint_idx"), &FABRIKSolver::get_joint_transform);

	ClassDB::bind_method(D_METHOD("set_joint_rotation_limit_enabled", "joint_idx", "joint_rotation_limit_enabled"), &FABRIKSolver::set_joint_rotation_limit_enabled);
	ClassDB::bind_method(D_METHOD("get_joint_rotation_limit_enabled", "joint_idx"), &FABRIKSolver::get_joint_rotation_limit_enabled);

	ClassDB::bind_method(D_METHOD("set_joint_rotation_limit_min", "joint_idx", "joint_rotation_limit_min"), &FABRIKSolver::set_joint_rotation_limit_min);
	ClassDB::bind_method(D_METHOD("get_joint_rotation_limit_min", "joint_idx"), &FABRIKSolver::get_joint_rotation_limit_min);

	ClassDB::bind_method(D_METHOD("set_joint_rotation_limit_max", "joint_idx", "joint_rotation_limit_max"), &FABRIKSolver::set_joint_rotation_limit_max);
	ClassDB::bind_method(D_METHOD("get_joint_rotation_limit_max", "joint_idx"), &FABRIKSolver::get_joint_rotation_limit_max);

	ClassDB::bind_method(D_METHOD("set_joint_hinge_enabled", "joint_idx", "joint_hinge_enabled"), &FABRIKSolver::set_joint_hinge_enabled);
	ClassDB::bind_method(D_METHOD("get_joint_hinge_enabled", "joint_idx"), &FABRIKSolver::get_joint_hinge_enabled);

	ClassDB::bind_method(D_METHOD("set_joint_hinge_axis", "joint_idx", "joint_hinge_axis"), &FABRIKSolver::set_joint_hinge_axis);
	ClassDB::bind_method(D_METHOD("get_joint_hinge_axis", "joint_idx"), &FABRIKSolver::get_joint_hinge_axis);

	ClassDB::bind_method(D_METHOD("set_use_pole_constraint", "use_pole_constraint"), &FABRIKSolver::set_use_pole_constraint);
	ClassDB::bind_method(D_METHOD("get_use_pole_constraint"), &FABRIKSolver::get_use_pole_constraint);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_pole_constraint"), "set_use_pole_constraint", "get_use_pole_constraint");

	ClassDB::bind_method(D_METHOD("set_pole_position", "pole_position"), &FABRIKSolver::set_pole_position);
	ClassDB::bind_method(D_METHOD("get_pole_position"), &FABRIKSolver::get_pole_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "pole_position"), "set_pole_position", "get_pole_position");

	ClassDB::bind_method(D_METHOD("set_target_position", "target_position"), &FABRIKSolver::set_target_position);
	ClassDB::bind_method(D_METHOD("get_target_position"), &FABRIKSolver::get_target_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target_position"), "set_target_position", "get_target_position");

	ClassDB::bind_method(D_METHOD("calculate_distances"), &FABRIKSolver::calculate_distances);
	ClassDB::bind_method(D_METHOD("get_debug_geo"), &FABRIKSolver::get_debug_geo);
	ClassDB::bind_method(D_METHOD("get_debug_geo2"), &FABRIKSolver::get_debug_geo);
	ClassDB::bind_method(D_METHOD("solve", "solve_iterations"), &FABRIKSolver::solve, DEFVAL(10));
}

void FABRIKSolver::_local_to_global() {
	Transform3D prev_global_trf;
	for (int i = 0; i < joints.size(); i++) {
		joints.write[i].global_transform = prev_global_trf * joints[i].transform;
		prev_global_trf = joints[i].global_transform;
	}
}

void FABRIKSolver::set_joint_count(int p_joint_count) {
	joints.resize_zeroed(p_joint_count);
}

int FABRIKSolver::get_joint_count() const {
	return joints.size();
}

void FABRIKSolver::set_joint_transform(int p_joint_idx, const Transform3D &p_transform) {
	ERR_FAIL_INDEX(p_joint_idx, joints.size());
	joints.write[p_joint_idx].transform = p_transform;
}

Transform3D FABRIKSolver::get_joint_transform(int p_joint_idx) const {
	ERR_FAIL_INDEX_V(p_joint_idx, joints.size(), Transform3D());
	return joints[p_joint_idx].transform;
}

void FABRIKSolver::set_joint_rotation_limit_enabled(int p_joint_idx, bool p_enable) {
	ERR_FAIL_INDEX(p_joint_idx, joints.size());
	joints.write[p_joint_idx].rotation_limit_enabled = p_enable;
}

bool FABRIKSolver::get_joint_rotation_limit_enabled(int p_joint_idx) const {
	ERR_FAIL_INDEX_V(p_joint_idx, joints.size(), false);
	return joints[p_joint_idx].rotation_limit_enabled;
}

void FABRIKSolver::set_joint_rotation_limit_min(int p_joint_idx, const Vector3 &p_rotation) {
	ERR_FAIL_INDEX(p_joint_idx, joints.size());
	joints.write[p_joint_idx].min_rotation = p_rotation;
}

Vector3 FABRIKSolver::get_joint_rotation_limit_min(int p_joint_idx) {
	ERR_FAIL_INDEX_V(p_joint_idx, joints.size(), Vector3());
	return joints[p_joint_idx].min_rotation;
}

void FABRIKSolver::set_joint_rotation_limit_max(int p_joint_idx, const Vector3 &p_rotation) {
	ERR_FAIL_INDEX(p_joint_idx, joints.size());
	joints.write[p_joint_idx].max_rotation = p_rotation;
}

Vector3 FABRIKSolver::get_joint_rotation_limit_max(int p_joint_idx) {
	ERR_FAIL_INDEX_V(p_joint_idx, joints.size(), Vector3());
	return joints[p_joint_idx].max_rotation;
}

void FABRIKSolver::calculate_distances() {
	full_chain_distance = 0.0f;
	for (int i = 1; i < joints.size(); i++) {
		joints.write[i].distance_to_parent = joints[i].transform.origin.length();
		full_chain_distance += joints[i].distance_to_parent;
	}
}

void FABRIKSolver::set_joint_hinge_enabled(int p_joint_idx, bool p_enable) {
	ERR_FAIL_INDEX(p_joint_idx, joints.size());
	joints.write[p_joint_idx].enable_hinge = p_enable;
}

bool FABRIKSolver::get_joint_hinge_enabled(int p_joint_idx) const {
	ERR_FAIL_INDEX_V(p_joint_idx, joints.size(), false);
	return joints[p_joint_idx].enable_hinge;
}

void FABRIKSolver::set_joint_hinge_axis(int p_joint_idx, const Vector3 &p_axis) {
	ERR_FAIL_INDEX(p_joint_idx, joints.size());
	joints.write[p_joint_idx].hinge_axis = p_axis;
}

Vector3 FABRIKSolver::get_joint_hinge_axis(int p_joint_idx) const {
	ERR_FAIL_INDEX_V(p_joint_idx, joints.size(), Vector3());
	return joints[p_joint_idx].hinge_axis;
}

void FABRIKSolver::solve(int p_iterations) {
	ZoneScopedN("FABRIK Solve");
	if (joints.size() == 0) {
		return;
	}

	float dist_eps_squared = DIST_EPS * DIST_EPS;

	if (use_pole_constraint) {
		_process_pole_vector();
	}

	_local_to_global();

	for (int i = 0; i < joints.size(); i++) {
		joints.write[i].working_position = joints[i].global_transform.origin;
	}

	if (joints[joints.size() - 1].working_position.distance_squared_to(target_position) < dist_eps_squared) {
		return;
	}

	Vector3 base_pos = joints[0].working_position;

	if (base_pos.distance_to(target_position) > full_chain_distance) {
		// If we are going to be fully extended just do the full extension now and be done with it
		for (int i = 1; i < joints.size(); i++) {
			Vector3 parent_position = joints[i - 1].working_position;
			joints.write[i].working_position = parent_position + parent_position.direction_to(target_position) * joints[i].distance_to_parent;
		}
		_apply_fabrik();
		return;
	}

	for (int i = 0; i < p_iterations; i++) {
		if (joints[joints.size() - 1].working_position.distance_squared_to(target_position) < dist_eps_squared) {
			_apply_fabrik();
			return;
		}
		_solve_backwards();
		_solve_forwards(base_pos);
		_apply_fabrik();
		_local_to_global();
	}
	_apply_fabrik();
}

Ref<ImmediateMesh> FABRIKSolver::get_debug_geo() const {
	return debug_geo;
}

FABRIKSolver::FABRIKSolver() {
	debug_geo.instantiate();
}

Vector3 FABRIKSolver::get_pole_position() const {
	return pole_position;
}

void FABRIKSolver::set_pole_position(const Vector3 &p_pole_position) {
	pole_position = p_pole_position;
}

bool FABRIKSolver::get_use_pole_constraint() const {
	return use_pole_constraint;
}

void FABRIKSolver::set_use_pole_constraint(bool p_use_pole_constraint) {
	use_pole_constraint = p_use_pole_constraint;
}

Vector3 FABRIKSolver::get_target_position() const {
	return target_position;
}

void FABRIKSolver::set_target_position(const Vector3 &p_target_position) {
	target_position = p_target_position;
}
