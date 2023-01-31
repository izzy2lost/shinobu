
#ifndef UTILS_H
#define UTILS_H

#include "core/math/vector3.h"
class HBUtils {
public:
	static float fast_negexp(float x) {
		return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
	}

	static float halflife_to_damping(float halflife, float eps = 1e-5f) {
		return (4.0f * 0.69314718056f) / (halflife + eps);
	}

	// Velocity spring, used for character.
	static void velocity_spring(
			float *v,
			float *a,
			float v_goal,
			float halflife,
			float dt) {
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
};

#endif // UTILS_H
