/**************************************************************************/
/*  inertialization.cpp                                                   */
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

#include "inertialization.h"
#include "animation_system/epas_animation.h"
#include "core/error/error_macros.h"
#include "core/math/quaternion.h"
#include "core/string/print_string.h"
#include "core/string/string_name.h"
#include <signal.h>

static String rot = "";

static float inertialize(float p_x0, float p_v0, float p_blend_time, float p_t) {
	if (p_v0 != 0) {
		float blend_time_1 = -5.0 * (p_x0 / p_v0);
		if (blend_time_1 > 0) {
			p_blend_time = MIN(blend_time_1, p_blend_time);
			p_t = MIN(p_blend_time, p_t);
		}
	}

	float bt_2 = p_blend_time * p_blend_time;
	float bt_3 = bt_2 * p_blend_time;
	float bt_4 = bt_3 * p_blend_time;
	float bt_5 = bt_4 * p_blend_time;
	float accel = MAX((-8.0f * p_v0 * p_blend_time - 20.0f * p_x0) / bt_2, 0.0f);
	float A = -((accel * bt_2 + 6.0f * p_v0 * p_blend_time + 12.0f * p_x0) / (2.0f * bt_5));
	float B = (3.0f * accel * bt_2 + 16.0f * p_v0 * p_blend_time + 30.0f * p_x0) / (2.0f * bt_4);
	float C = -((3.0f * accel * bt_2 + 12.0f * p_v0 * p_blend_time + 20.0f * p_x0) / (2.0f * bt_3));

	return A * Math::pow(p_t, 5.0f) + B * Math::pow(p_t, 4.0f) + C * Math::pow(p_t, 3.0f) + (accel * 0.5f) * Math::pow(p_t, 2.0f) + p_v0 * p_t + p_x0;
}

void RotationInertializer::_bind_methods() {
	ClassDB::bind_static_method("RotationInertializer", D_METHOD("create", "prev_prev", "prev", "target", "transition_time", "delta"), &RotationInertializer::create);
	ClassDB::bind_method(D_METHOD("advance", "delta"), &RotationInertializer::advance);
	ClassDB::bind_method(D_METHOD("advance_cock", "delta"), &RotationInertializer::advance_cock);
	ClassDB::bind_method(D_METHOD("is_done"), &RotationInertializer::is_done);
}

static bool hack = false;

Quaternion RotationInertializer::advance_cock(float p_delta) {
	hack = true;
	Quaternion cock = advance(p_delta);
	hack = false;

	return cock;
}
Quaternion RotationInertializer::advance(float p_delta) {
	current_transition_time += p_delta;
	current_transition_time = MIN(current_transition_time, transition_duration);

	float rot_x = inertialize(rotation_offset_angle, rotation_velocity, transition_duration, current_transition_time);
	rot_x = MAX(rot_x, 0.0f);
	rot_x = MIN(rot_x, rotation_offset_angle);
	if (rot_x > rotation_offset_angle) {
		raise(SIGTRAP);
	}
	DEV_ASSERT(rotation_offset_angle >= -0.001f);
	Quaternion rot_off = Quaternion(rotation_offset_axis, rot_x);
	return rot_off;
}

bool RotationInertializer::is_done() const {
	return current_transition_time >= transition_duration;
}

float RotationInertializer::get_offset_angle() const {
	return rotation_offset_angle;
}

Vector3 RotationInertializer::get_offset_axis() const {
	return rotation_offset_axis;
}

void quat_to_angle_axis(Quaternion q, float &angle, Vector3 &axis, float eps = 1e-8f) {
	float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);

	if (length < eps) {
		angle = 0.0f;
		axis = Vector3(1.0f, 0.0f, 0.0f);
	} else {
		angle = 2.0f * acosf(CLAMP(q.w, -1.0f, 1.0f));
		axis = Vector3(q.x, q.y, q.z) / length;
	}
}

Ref<RotationInertializer> RotationInertializer::create(const Quaternion &p_prev_prev, const Quaternion &p_prev, const Quaternion &p_target, float p_duration, float p_delta) {
	if (p_prev.angle_to(p_target) < Math::deg_to_rad(0.05f)) {
		return nullptr;
	}

	Ref<RotationInertializer> in;
	in.instantiate();

	Quaternion q_prev = p_target.inverse() * p_prev;
	q_prev.normalize();
	Quaternion q_prev_prev = p_target.inverse() * p_prev_prev;
	Vector3 x0_axis;
	float x0_angle;

	// We have to use this function because godot's built in angle axis decomposition thing is entirely broken
	// and may sometimes return NaNs
	quat_to_angle_axis(q_prev, x0_angle, x0_axis);

	x0_axis.normalize();

	// Ensure that rotations are the shortest possible
	if (x0_angle > Math_PI) {
		x0_angle = 2.0f * Math_PI - x0_angle;
		x0_axis = -x0_axis;
	}

	Vector3 q_x_y_z = Vector3(q_prev_prev.x, q_prev_prev.y, q_prev_prev.z);
	float q_x_m_1 = 2.0f * Math::atan(q_x_y_z.dot(x0_axis) / q_prev_prev.w);
	in->rotation_velocity = MIN((x0_angle - q_x_m_1) / p_delta, 0.0);
	in->rotation_offset_angle = x0_angle;
	in->rotation_offset_axis = x0_axis;
	in->transition_duration = p_duration;

	return in;
}

Vector3 PositionInertializer::advance(float p_delta) {
	current_transition_time += p_delta;
	current_transition_time = MIN(transition_duration, current_transition_time);
	float pos_x = inertialize(position_offset.length(), position_velocity, transition_duration, current_transition_time);
	Vector3 pos_off = position_offset.normalized() * pos_x;
	return pos_off;
}

bool PositionInertializer::is_done() const {
	return current_transition_time >= transition_duration;
}

Vector3 PositionInertializer::get_offset() const {
	return position_offset;
}

Ref<PositionInertializer> PositionInertializer::create(const Vector3 &p_prev_prev, const Vector3 &p_prev, const Vector3 &p_target, float p_duration, float p_delta) {
	Ref<PositionInertializer> in;
	in.instantiate();

	// Position info
	Vector3 x_prev = p_prev - p_target;
	Vector3 x_prev_prev = p_prev_prev - p_target;
	float x_m_1 = x_prev_prev.dot(x_prev.normalized());
	in->position_velocity = MIN((x_prev.length() - x_m_1) / p_delta, 0.0f);
	in->position_offset = x_prev;
	in->transition_duration = p_duration;
	if (in->position_velocity != 0.0f) {
		in->transition_duration = MIN(p_duration, -5.0f * (x_prev.length() / in->position_velocity));
	}
	return in;
}

float EPASPoseInertializer::get_current_transition_time() const {
	return current_transition_time;
}

bool EPASPoseInertializer::advance(Ref<EPASPose> p_target_pose, const Ref<EPASPose> &p_base_pose, float p_delta) {
	bool done = true;
	current_transition_time += p_delta;
	for (int i = 0; i < transition_infos.size(); i++) {
		StringName bone_name = transition_infos[i].bone_name;

		if (!p_target_pose->has_bone(bone_name)) {
			p_target_pose->create_bone(bone_name);
		}

		if (transition_infos[i].position_inertializer.is_valid() && !transition_infos[i].position_inertializer->is_done()) {
			Ref<PositionInertializer> pos_inertializer = transition_infos[i].position_inertializer;
			Vector3 bone_pos = p_target_pose->get_bone_position(bone_name, p_base_pose) + pos_inertializer->advance(p_delta);
			p_target_pose->set_bone_position(bone_name, bone_pos);
			done = !pos_inertializer->is_done() ? false : done;
		}
		if (transition_infos[i].rotation_inertializer.is_valid() && !transition_infos[i].rotation_inertializer->is_done()) {
			if (transition_infos[i].bone_name == StringName("thigh.R")) {
				hack = true;
			}
			Ref<RotationInertializer> rot_inertializer = transition_infos[i].rotation_inertializer;
			Quaternion bone_rot = p_target_pose->get_bone_rotation(bone_name, p_base_pose) * rot_inertializer->advance(p_delta);
			p_target_pose->set_bone_rotation(bone_name, bone_rot);
			done = !rot_inertializer->is_done() ? false : done;
			if (transition_infos[i].bone_name == StringName("thigh.R")) {
				hack = false;
			}
		}
	}
	return done;
}

Ref<EPASPoseInertializer> EPASPoseInertializer::create(const Ref<EPASPose> p_poses[POSE_MAX], const Ref<EPASPose> &p_base_pose, float p_transition_duration, float p_delta, TypedArray<StringName> p_bone_filter) {
	ERR_FAIL_COND_V_MSG(!p_poses[InertializationPose::PREV_PREV_POSE].is_valid(), Ref<EPASPoseInertializer>(), "No previous previous pose was given");
	ERR_FAIL_COND_V_MSG(!p_poses[InertializationPose::PREV_POSE].is_valid(), Ref<EPASPoseInertializer>(), "No previous pose was given");
	ERR_FAIL_COND_V_MSG(!p_poses[InertializationPose::TARGET_POSE].is_valid(), Ref<EPASPoseInertializer>(), "No current pose was given");
	Ref<EPASPoseInertializer> in;
	in.instantiate();
	Vector<EPASPoseInertializer::TransitionInfo> transition_infos;

	Ref<EPASPose> prev_prev_pose = p_poses[InertializationPose::PREV_PREV_POSE];
	Ref<EPASPose> prev_pose = p_poses[InertializationPose::PREV_POSE];
	Ref<EPASPose> target_pose = p_poses[InertializationPose::TARGET_POSE];

	for (int i = 0; i < p_base_pose->get_bone_count(); i++) {
		StringName bone_name = p_base_pose->get_bone_name(i);
		if (p_bone_filter.size() > 0 && !p_bone_filter.has(bone_name)) {
			continue;
		}
		EPASPoseInertializer::TransitionInfo ti;
		ti.bone_name = bone_name;

		// Position setup
		Vector3 prev_prev_bone_pos = prev_prev_pose->get_bone_position(bone_name, p_base_pose);
		Vector3 prev_bone_pos = prev_pose->get_bone_position(bone_name, p_base_pose);
		Vector3 target_bone_pos = target_pose->get_bone_position(bone_name, p_base_pose);
		Ref<PositionInertializer> pos_inertializer = PositionInertializer::create(prev_prev_bone_pos, prev_bone_pos, target_bone_pos, p_transition_duration, p_delta);
		if (!Math::is_zero_approx(pos_inertializer->get_offset().length())) {
			ti.position_inertializer = pos_inertializer;
		}

		Quaternion prev_prev_bone_rot = prev_prev_pose->get_bone_rotation(bone_name, p_base_pose);
		Quaternion prev_bone_rot = prev_pose->get_bone_rotation(bone_name, p_base_pose);
		Quaternion target_bone_rot = target_pose->get_bone_rotation(bone_name, p_base_pose);
		Ref<RotationInertializer> rot_inertializer = RotationInertializer::create(prev_prev_bone_rot, prev_bone_rot, target_bone_rot, p_transition_duration, p_delta);
		ti.rotation_inertializer = rot_inertializer;

		if (ti.rotation_inertializer.is_valid() && ti.rotation_inertializer->transition_duration == 0.0f) {
			ti.rotation_inertializer = Ref<RotationInertializer>();
		}

		if (ti.position_inertializer.is_valid() || ti.rotation_inertializer.is_valid()) {
			transition_infos.push_back(ti);
		}
	}

	in->transition_infos = transition_infos;
	return in;
}
