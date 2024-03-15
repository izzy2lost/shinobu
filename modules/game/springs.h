/**************************************************************************/
/*  springs.h                                                             */
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

#ifndef SPRINGS_H
#define SPRINGS_H

#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/math/vector2.h"
#include "core/math/vector3.h"
#include "core/string/print_string.h"

class HBSprings {
public:
	static real_t fast_negexp(real_t x) {
		return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
	}

	static real_t halflife_to_damping(real_t halflife, real_t eps = 1e-5f) {
		return (4.0f * 0.69314718056f) / (halflife + eps);
	}

	static void velocity_spring(
			real_t *v,
			real_t *a,
			real_t v_goal,
			real_t halflife,
			real_t dt) {
		// Velocity spring, used for character.
		real_t y = halflife_to_damping(halflife) / 2.0f;
		real_t j0 = *v - v_goal;
		real_t j1 = *a + j0 * y;
		real_t eydt = fast_negexp(y * dt);

		*v = eydt * (j0 + j1 * dt) + v_goal;
		*a = eydt * (*a - j1 * y * dt);
	}

	static void velocity_spring_vector2(
			Vector2 &v,
			Vector2 &a,
			const Vector2 &v_goal,
			real_t halflife,
			real_t dt) {
		velocity_spring(&(v.x), &(a.x), v_goal.x, halflife, dt);
		velocity_spring(&(v.y), &(a.y), v_goal.y, halflife, dt);
	}

	static void velocity_spring_vector3(
			Vector3 &v,
			Vector3 &a,
			const Vector3 &v_goal,
			real_t halflife,
			real_t dt) {
		velocity_spring(&(v.x), &(a.x), v_goal.x, halflife, dt);
		velocity_spring(&(v.y), &(a.y), v_goal.y, halflife, dt);
		velocity_spring(&(v.z), &(a.z), v_goal.z, halflife, dt);
	}

	static Vector3 quat_to_scaled_angle_axis(Quaternion q) {
		return 2.0f * (q.get_axis().normalized() * q.get_angle());
	}

	static Quaternion quat_from_scaled_angle_axis(Vector3 v) {
		return (Quaternion(v.x, v.y, v.z, 0.0f) / 2.0f).exp();
	}

	static void simple_spring_damper_exact_quat(
			Quaternion &x,
			Vector3 &v,
			const Quaternion &x_goal,
			real_t halflife,
			real_t dt) {
		real_t y = halflife_to_damping(halflife) / 2.0f;

		Quaternion goal = x_goal;

		real_t dot = goal.dot(x);

		if (dot < 0.0f) {
			goal = -goal;
		}

		Vector3 j0 = quat_to_scaled_angle_axis(goal.inverse() * x);
		Vector3 j1 = v + j0 * y;

		real_t eydt = fast_negexp(y * dt);

		Quaternion q = quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt));
		x = goal * q;
		v = eydt * (v - j1 * y * dt);
	}

	static void critical_spring_damper_exact(
			real_t *x,
			real_t *v,
			real_t x_goal,
			real_t halflife,
			real_t dt) {
		real_t y = halflife_to_damping(halflife) / 2.0f;
		real_t j0 = *x - x_goal;
		real_t j1 = *v + j0 * y;
		real_t eydt = fast_negexp(y * dt);

		*x = eydt * (j0 + j1 * dt) + x_goal;
		*v = eydt * (*v - j1 * y * dt);
	}

	static void critical_spring_damper_exact_vector3(
			Vector3 &x,
			Vector3 &v,
			Vector3 x_goal,
			real_t halflife,
			real_t dt) {
		critical_spring_damper_exact(&(x.x), &(v.x), x_goal.x, halflife, dt);
		critical_spring_damper_exact(&(x.y), &(v.y), x_goal.y, halflife, dt);
		critical_spring_damper_exact(&(x.z), &(v.z), x_goal.z, halflife, dt);
	}

	static real_t squaref(real_t x) {
		return x * x;
	}

	static real_t damping_ratio_to_stiffness(real_t ratio, real_t damping) {
		return squaref(damping / (ratio * 2.0f));
	}

	static void spring_damper_exact_stiffness_damping(
			real_t &x,
			real_t &v,
			real_t x_goal,
			real_t v_goal,
			real_t stiffness,
			real_t damping,
			real_t dt,
			real_t eps = 1e-5f) {
		real_t g = x_goal;
		real_t q = v_goal;
		real_t s = stiffness;
		real_t d = damping;
		real_t c = g + (d * q) / (s + eps);
		real_t y = d / 2.0f;

		if (fabs(s - (d * d) / 4.0f) < eps) // Critically Damped
		{
			real_t j0 = x - c;
			real_t j1 = v + j0 * y;

			real_t eydt = fast_negexp(y * dt);

			x = j0 * eydt + dt * j1 * eydt + c;
			v = -y * j0 * eydt - y * dt * j1 * eydt + j1 * eydt;
		} else if (s - (d * d) / 4.0f > 0.0) // Under Damped
		{
			real_t w = sqrtf(s - (d * d) / 4.0f);
			real_t j = sqrtf(squaref(v + y * (x - c)) / (w * w + eps) + squaref(x - c));
			real_t p = Math::atan((v + (x - c) * y) / (-(x - c) * w + eps));

			j = (x - c) > 0.0f ? j : -j;

			real_t eydt = fast_negexp(y * dt);

			x = j * eydt * cosf(w * dt + p) + c;
			v = -y * j * eydt * cosf(w * dt + p) - w * j * eydt * sinf(w * dt + p);
		} else if (s - (d * d) / 4.0f < 0.0) // Over Damped
		{
			real_t y0 = (d + sqrtf(d * d - 4 * s)) / 2.0f;
			real_t y1 = (d - sqrtf(d * d - 4 * s)) / 2.0f;
			real_t j1 = (c * y0 - x * y0 - v) / (y1 - y0);
			real_t j0 = x - j1 - c;

			real_t ey0dt = fast_negexp(y0 * dt);
			real_t ey1dt = fast_negexp(y1 * dt);

			x = j0 * ey0dt + j1 * ey1dt + c;
			v = -y0 * j0 * ey0dt - y1 * j1 * ey1dt;
		}
	}

	static void spring_damper_exact_ratio(
			real_t &x,
			real_t &v,
			real_t x_goal,
			real_t v_goal,
			real_t damping_ratio,
			real_t halflife,
			real_t dt,
			real_t eps = 1e-5f) {
		real_t g = x_goal;
		real_t q = v_goal;
		real_t d = halflife_to_damping(halflife);
		real_t s = damping_ratio_to_stiffness(damping_ratio, d);
		real_t c = g + (d * q) / (s + eps);
		real_t y = d / 2.0f;

		if (fabs(s - (d * d) / 4.0f) < eps) // Critically Damped
		{
			real_t j0 = x - c;
			real_t j1 = v + j0 * y;

			real_t eydt = fast_negexp(y * dt);

			x = j0 * eydt + dt * j1 * eydt + c;
			v = -y * j0 * eydt - y * dt * j1 * eydt + j1 * eydt;
		} else if (s - (d * d) / 4.0f > 0.0) // Under Damped
		{
			real_t w = Math::sqrt(s - (d * d) / 4.0f);
			real_t j = Math::sqrt(squaref(v + y * (x - c)) / (w * w + eps) + squaref(x - c));
			real_t p = Math::atan((v + (x - c) * y) / (-(x - c) * w + eps));

			j = (x - c) > 0.0f ? j : -j;

			real_t eydt = fast_negexp(y * dt);

			x = j * eydt * cosf(w * dt + p) + c;
			v = -y * j * eydt * cosf(w * dt + p) - w * j * eydt * sinf(w * dt + p);
		} else if (s - (d * d) / 4.0f < 0.0) // Over Damped
		{
			real_t y0 = (d + sqrtf(d * d - 4 * s)) / 2.0f;
			real_t y1 = (d - sqrtf(d * d - 4 * s)) / 2.0f;
			real_t j1 = (c * y0 - x * y0 - v) / (y1 - y0);
			real_t j0 = x - j1 - c;

			real_t ey0dt = fast_negexp(y0 * dt);
			real_t ey1dt = fast_negexp(y1 * dt);

			x = j0 * ey0dt + j1 * ey1dt + c;
			v = -y0 * j0 * ey0dt - y1 * j1 * ey1dt;
		}
	}

	static void spring_damper_exact_ratio_vector3(
			Vector3 &x,
			Vector3 &v,
			Vector3 x_goal,
			Vector3 v_goal,
			real_t damping_ratio,
			real_t halflife,
			real_t dt,
			real_t eps = 1e-5f) {
		spring_damper_exact_ratio(x.x, v.x, x_goal.x, v_goal.x, damping_ratio, halflife, dt, eps);
		spring_damper_exact_ratio(x.y, v.y, x_goal.y, v_goal.y, damping_ratio, halflife, dt, eps);
		spring_damper_exact_ratio(x.z, v.z, x_goal.z, v_goal.z, damping_ratio, halflife, dt, eps);
	}

	static void tracking_spring_update_no_velocity_acceleration_exact(
			real_t &x,
			real_t &v,
			real_t x_goal,
			real_t x_gain,
			real_t dt,
			real_t gain_dt) {
		real_t t0 = 1.0f - x_gain;
		real_t t3 = x_gain / (gain_dt * gain_dt);

		real_t stiffness = t3;
		real_t damping = (1.0f - t0) / gain_dt;
		real_t spring_x_goal = x_goal;
		real_t spring_v_goal = 0.0f;

		spring_damper_exact_stiffness_damping(
				x,
				v,
				spring_x_goal,
				spring_v_goal,
				stiffness,
				damping,
				dt);
	}
	real_t tracking_target_acceleration(
			real_t x_next,
			real_t x_curr,
			real_t x_prev,
			real_t dt) {
		return (((x_next - x_curr) / dt) - ((x_curr - x_prev) / dt)) / dt;
	}

	real_t tracking_target_velocity(
			real_t x_next,
			real_t x_curr,
			real_t dt) {
		return (x_next - x_curr) / dt;
	}

	static Vector3 tracking_target_acceleration(
			Vector3 x_next,
			Vector3 x_curr,
			Vector3 x_prev,
			real_t dt) {
		return (((x_next - x_curr) / dt) - ((x_curr - x_prev) / dt)) / dt;
	}

	static Vector3 tracking_target_velocity(
			Vector3 x_next,
			Vector3 x_curr,
			real_t dt) {
		return (x_next - x_curr) / dt;
	}

	static void tracking_spring_update_exact(
			real_t &x,
			real_t &v,
			real_t x_goal,
			real_t v_goal,
			real_t a_goal,
			real_t x_gain,
			real_t v_gain,
			real_t a_gain,
			real_t dt,
			real_t gain_dt) {
		real_t t0 = (1.0f - v_gain) * (1.0f - x_gain);
		real_t t1 = a_gain * (1.0f - v_gain) * (1.0f - x_gain);
		real_t t2 = (v_gain * (1.0f - x_gain)) / gain_dt;
		real_t t3 = x_gain / (gain_dt * gain_dt);

		real_t stiffness = t3;
		real_t damping = (1.0f - t0) / gain_dt;
		real_t spring_x_goal = x_goal;
		real_t spring_v_goal = (t2 * v_goal + t1 * a_goal) / ((1.0f - t0) / gain_dt);

		spring_damper_exact_stiffness_damping(
				x,
				v,
				spring_x_goal,
				spring_v_goal,
				stiffness,
				damping,
				dt);
	}

	static void tracking_spring_update_exact_vector3(
			Vector3 &x,
			Vector3 &v,
			Vector3 x_goal,
			Vector3 v_goal,
			Vector3 a_goal,
			real_t x_gain,
			real_t v_gain,
			real_t a_gain,
			real_t dt,
			real_t gain_dt) {
		tracking_spring_update_exact(x.x, v.x, x_goal.x, v_goal.x, a_goal.x, x_gain, v_gain, a_gain, dt, gain_dt);
		tracking_spring_update_exact(x.y, v.y, x_goal.y, v_goal.y, a_goal.y, x_gain, v_gain, a_gain, dt, gain_dt);
		tracking_spring_update_exact(x.z, v.z, x_goal.z, v_goal.z, a_goal.z, x_gain, v_gain, a_gain, dt, gain_dt);
	}
};
#endif // SPRINGS_H
