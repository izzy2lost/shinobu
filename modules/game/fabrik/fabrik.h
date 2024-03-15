/**************************************************************************/
/*  fabrik.h                                                              */
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

#ifndef FABRIK_H
#define FABRIK_H

#include "core/object/ref_counted.h"
#include "scene/resources/immediate_mesh.h"

class FABRIKSolver : public RefCounted {
	GDCLASS(FABRIKSolver, RefCounted);

	const float DIST_EPS = 0.0001f;

	Ref<ImmediateMesh> debug_geo;

	struct FABRIKJoint {
		Transform3D transform;
		Transform3D global_transform;
		Vector3 working_position;
		float distance_to_parent = 0.0f;

		bool enable_hinge = false;
		Vector3 hinge_axis = Vector3(1.0f, 0.0f, 0.0f);

		bool rotation_limit_enabled = false;
		Vector3 min_rotation = Vector3(-Math_PI, -Math_PI, -Math_PI);
		Vector3 max_rotation = Vector3(Math_PI, Math_PI, Math_PI);
	};

	Vector<FABRIKJoint> joints;
	Vector3 pole_position;
	Vector3 target_position;
	bool use_pole_constraint = false;
	float full_chain_distance = 0.0f;

	void _solve_backwards();
	void _solve_forwards(const Vector3 &p_base);
	void _process_pole_vector();
	void _local_to_global();
	void _apply_fabrik();

protected:
	static void _bind_methods();

public:
	void set_joint_count(int p_joint_count);
	int get_joint_count() const;

	void set_joint_transform(int p_joint_idx, const Transform3D &p_transform);
	Transform3D get_joint_transform(int p_joint_idx) const;

	void set_joint_rotation_limit_enabled(int p_joint_idx, bool p_enable);
	bool get_joint_rotation_limit_enabled(int p_joint_idx) const;
	void set_joint_rotation_limit_min(int p_joint_idx, const Vector3 &p_rotation);
	Vector3 get_joint_rotation_limit_min(int p_joint_idx);
	void set_joint_rotation_limit_max(int p_joint_idx, const Vector3 &p_rotation);
	Vector3 get_joint_rotation_limit_max(int p_joint_idx);

	void calculate_distances();
	void set_joint_hinge_enabled(int p_joint_idx, bool p_enable);
	bool get_joint_hinge_enabled(int p_joint_idx) const;
	void set_joint_hinge_axis(int p_joint_idx, const Vector3 &p_axis);
	Vector3 get_joint_hinge_axis(int p_joint_idx) const;

	bool get_use_pole_constraint() const;
	void set_use_pole_constraint(bool p_use_pole_constraint);

	Vector3 get_target_position() const;
	void set_target_position(const Vector3 &p_target_position);

	Vector3 get_pole_position() const;
	void set_pole_position(const Vector3 &p_pole_position);

	void solve(int p_iterations = 10);

	Ref<ImmediateMesh> get_debug_geo() const;

	Transform3D get_global_trf(int p_idx);

	FABRIKSolver();
};

class FABRIKLimbSolver : public FABRIKSolver {
public:
	FABRIKLimbSolver() :
			FABRIKSolver() {
		set_joint_count(3);
		set_joint_hinge_enabled(1, true);
		set_use_pole_constraint(true);
	}
};

#endif // FABRIK_H
