
#ifndef UTILS_H
#define UTILS_H

#include "core/error/error_macros.h"
#include "core/math/projection.h"
#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/object/class_db.h"
#include "core/object/ref_counted.h"
#include "core/variant/array.h"
class HBUtils : public RefCounted {
	GDCLASS(HBUtils, RefCounted);

protected:
	static void _bind_methods() {
		ClassDB::bind_static_method("HBUtils", D_METHOD("rotate_to_align", "start_direction", "end_direction"), &HBUtils::rotate_to_align);
	};

public:
	static float fast_negexp(float x) {
		return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
	}

	static float halflife_to_damping(float halflife, float eps = 1e-5f) {
		return (4.0f * 0.69314718056f) / (halflife + eps);
	}

	static void velocity_spring(
			float *v,
			float *a,
			float v_goal,
			float halflife,
			float dt) {
		// Velocity spring, used for character.
		float y = halflife_to_damping(halflife) / 2.0f;
		float j0 = *v - v_goal;
		float j1 = *a + j0 * y;
		float eydt = fast_negexp(y * dt);

		*v = eydt * (j0 + j1 * dt) + v_goal;
		*a = eydt * (*a - j1 * y * dt);
	}

	static void velocity_spring_vector3(
			Vector3 &v,
			Vector3 &a,
			const Vector3 &v_goal,
			float halflife,
			float dt) {
		velocity_spring(&(v.x), &(a.x), v_goal.x, halflife, dt);
		velocity_spring(&(v.y), &(a.y), v_goal.y, halflife, dt);
		velocity_spring(&(v.z), &(a.z), v_goal.z, halflife, dt);
	}

	static void rotate_normal_towards(Vector3 &p_pos, const Vector3 &p_goal, real_t p_angle_delta) {
		// rotates p_pos towards p_goal without overshooting by p_angle_delta (in radians)
		real_t angle = p_pos.angle_to(p_goal);
		real_t percentage = CLAMP(p_angle_delta / angle, -1.0f, 1.0f);
		percentage = Math::abs(percentage);
		p_pos = p_pos.slerp(p_goal, percentage);
	}

	static Vector3 quat_to_scaled_angle_axis(Quaternion q) {
		return 2.0f * (q.get_axis() * q.get_angle());
	}

	static Quaternion quat_from_scaled_angle_axis(Vector3 v) {
		return (Quaternion(v.x, v.y, v.z, 0.0f) / 2.0f).exp();
	}

	static void simple_spring_damper_exact_quat(
			Quaternion &x,
			Vector3 &v,
			const Quaternion &x_goal,
			float halflife,
			float dt) {
		float y = halflife_to_damping(halflife) / 2.0f;

		Vector3 j0 = quat_to_scaled_angle_axis(x_goal.inverse() * x);
		Vector3 j1 = v + j0 * y;

		float eydt = fast_negexp(y * dt);

		x = x_goal * quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt));
		v = eydt * (v - j1 * y * dt);
	}

	static void trf_to_mat(const Transform3D &p_mat, float *p_out) {
		p_out[0] = p_mat.basis.rows[0][0];
		p_out[1] = p_mat.basis.rows[1][0];
		p_out[2] = p_mat.basis.rows[2][0];
		p_out[3] = 0;

		p_out[4] = p_mat.basis.rows[0][1];
		p_out[5] = p_mat.basis.rows[1][1];
		p_out[6] = p_mat.basis.rows[2][1];
		p_out[7] = 0;

		p_out[8] = p_mat.basis.rows[0][2];
		p_out[9] = p_mat.basis.rows[1][2];
		p_out[10] = p_mat.basis.rows[2][2];
		p_out[11] = 0;

		p_out[12] = p_mat.origin.x;
		p_out[13] = p_mat.origin.y;
		p_out[14] = p_mat.origin.z;
		p_out[15] = 1;
	}

	static void mat_to_trf(const float *p_in, Transform3D &p_out) {
		p_out.basis.rows[0][0] = p_in[0];
		p_out.basis.rows[1][0] = p_in[1];
		p_out.basis.rows[2][0] = p_in[2];

		p_out.basis.rows[0][1] = p_in[4];
		p_out.basis.rows[1][1] = p_in[5];
		p_out.basis.rows[2][1] = p_in[6];

		p_out.basis.rows[0][2] = p_in[8];
		p_out.basis.rows[1][2] = p_in[9];
		p_out.basis.rows[2][2] = p_in[10];

		p_out.origin.x = p_in[12];
		p_out.origin.y = p_in[13];
		p_out.origin.z = p_in[14];
	}

	static void proj_to_mat(const Projection &p_mat, float *p_out) {
		for (int i = 0; i < 16; i++) {
			real_t *p = (real_t *)p_mat.columns;
			p_out[i] = p[i];
		}
	}

	static Basis rotate_to_align(Vector3 p_start_direction, Vector3 p_end_direction) {
		Basis basis;
		basis.rotate_to_align(p_start_direction, p_end_direction);
		return basis;
	}

	static void two_joint_ik(
			Vector3 pos_a, Vector3 pos_b, Vector3 pos_c, Vector3 p_target, float eps,
			Quaternion p_a_global_rot, Quaternion p_b_global_rot,
			Quaternion &p_a_local_rot, Quaternion &p_b_local_rot,
			bool p_use_magnet = false, Vector3 p_magnet_pos = Vector3()) {
		// Linear algebra based two joint IK
		// Notes about this function:
		// You want to res
		float lab = pos_b.distance_to(pos_a);
		float lcb = pos_b.distance_to(pos_c);
		float lat = CLAMP(p_target.distance_to(pos_a), eps, lab + lcb - eps);

		// Use cosine rule to calculate the current angles
		float ac_ab_0 = Math::acos(CLAMP(pos_a.direction_to(pos_c).dot(pos_a.direction_to(pos_b)), -1.0f, 1.0f));
		float ba_bc_0 = Math::acos(CLAMP(pos_c.direction_to(pos_b).dot(pos_a.direction_to(pos_b)), -1.0f, 1.0f));
		float ac_at_0 = Math::acos(CLAMP(pos_a.direction_to(pos_c).dot(pos_a.direction_to(p_target)), -1.0f, 1.0f));

		// Calculate the desired angles
		float ac_ab_1 = Math::acos(CLAMP((lcb * lcb - lab * lab - lat * lat) / (-2.0f * lab * lat), -1.0f, 1.0f));
		float ba_bc_1 = Math::acos(CLAMP((lat * lat - lab * lab - lcb * lcb) / (-2.0f * lab * lcb), -1.0f, 1.0f));

		// Calculate the axis of rotation for lengthening/shortening the chain
		Vector3 axis0 = pos_a.direction_to(pos_c).cross(pos_a.direction_to(pos_c)).normalized();
		if (p_use_magnet) {
			Vector3 dir = pos_b.direction_to(p_magnet_pos);
			axis0 = pos_a.direction_to(pos_c).cross(dir).normalized();
		}
		// Calculate the axis of rotation for aligning the chain with the target
		Vector3 axis1 = pos_a.direction_to(pos_c).cross(pos_a.direction_to(p_target)).normalized();

		// Hip rotation
		Quaternion r0 = Quaternion(p_a_global_rot.inverse().xform(axis0), ac_ab_1 - ac_ab_0);
		// Knee rotation
		Quaternion r1 = Quaternion(p_b_global_rot.inverse().xform(axis0), ba_bc_1 - ba_bc_0);
		// Additional hip rotation to align
		Quaternion r2 = Quaternion(p_a_global_rot.inverse().xform(axis1), ac_at_0);

		p_a_local_rot = p_a_local_rot * (r2 * r0);
		p_b_local_rot = p_b_local_rot * r1;
	}

	static void get_cubic_spline_weights(float interp, float *weights) {
		// Lifted straight from overgrwoth
		float interp_squared = interp * interp;
		float interp_cubed = interp_squared * interp;
		weights[0] = 0.5f * (-interp_cubed + 2.0f * interp_squared - interp);
		weights[1] = 0.5f * (3.0f * interp_cubed - 5.0f * interp_squared + 2.0f);
		weights[2] = 0.5f * (-3.0f * interp_cubed + 4.0f * interp_squared + interp);
		weights[3] = 0.5f * (interp_cubed - interp_squared);
	}

	static float inertialize(float p_x0, float p_v0, float p_blend_time, float p_t) {
		float accel = MAX(-8.0f * p_v0 * p_blend_time - 20.0f * p_x0, 0.0f);
		float A = -((accel * Math::pow(p_blend_time, 2.0f) + 6.0f * p_v0 * p_blend_time + 12.0f * p_x0) / (2.0f * Math::pow(p_blend_time, 5.0f)));
		float B = (3.0f * accel * Math::pow(p_blend_time, 2.0f) + 16.0f * p_v0 * p_blend_time + 30.0f * p_x0) / (2.0f * Math::pow(p_blend_time, 4.0f));
		float C = -((3.0f * accel * Math::pow(p_blend_time, 2.0f) + 12.0f * p_v0 * p_blend_time + 20.0f * p_x0) / (2.0f * Math::pow(p_blend_time, 3.0f)));

		return A * Math::pow(p_t, 5.0f) + B * Math::pow(p_t, 4.0f) + C * Math::pow(p_t, 3.0f) + (accel * 0.5f) * Math::pow(p_t, 2.0f) + p_v0 * p_t + p_x0;
	}
};

#endif // UTILS_H
