/**************************************************************************/
/*  agent_procedural_anim.cpp                                             */
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

#include "agent_procedural_anim.h"
#include "springs.h"

void AgentProceduralAnimator::_advance_animation(AgentProceduralAnimOptions &p_options, float p_delta) {
	_process_springs(p_options, p_delta);
}

void AgentProceduralAnimator::_process_springs(AgentProceduralAnimOptions &p_options, float p_delta) {
	if (p_delta == 0.0f) {
		return;
	}
	// Process limb springs
	for (int i = 0; i < LIMB_MAX; i++) {
		Transform3D trf = animator_state.limb_output_transforms[i];
		float position_spring_halflife = p_options.limb_position_spring_halflifes[i];
		if (position_spring_halflife <= 0.0f) {
			trf.origin = animator_state.limb_transforms[i].origin;
		} else {
			HBSprings::critical_spring_damper_exact_vector3(
					trf.origin,
					animator_state.limb_position_spring_velocities[i],
					animator_state.limb_transforms[i].origin,
					position_spring_halflife,
					p_delta);
		}

		float rotation_spring_halflife = p_options.limb_rotation_spring_halflifes[i];

		if (rotation_spring_halflife <= 0.0f) {
			trf.basis = animator_state.limb_transforms[i].basis;
		} else {
			Quaternion rot = trf.basis.get_rotation_quaternion();
			HBSprings::simple_spring_damper_exact_quat(
					rot,
					animator_state.limb_rotation_spring_velocities[i],
					animator_state.limb_transforms[i].basis.get_rotation_quaternion(),
					rotation_spring_halflife,
					p_delta);
			trf.basis = rot;
		}

		animator_state.limb_output_transforms[i] = trf;
	}

	// Process skeleton springs
	Transform3D skeleton_trf = animator_state.skeleton_output_transform;
	if (p_options.skeleton_rotation_spring_halflife <= 0.0f) {
		skeleton_trf.basis = animator_state.skeleton_transform.basis;
	} else {
		Quaternion rot = animator_state.skeleton_output_transform.basis.get_rotation_quaternion();
		HBSprings::simple_spring_damper_exact_quat(
				rot,
				animator_state.skeleton_rotation_spring_velocity,
				animator_state.skeleton_transform.basis.get_rotation_quaternion(),
				p_options.skeleton_rotation_spring_halflife,
				p_delta);
		skeleton_trf.basis = rot;
	}

	if (p_options.skeleton_position_spring_halflife <= 0.0f) {
		skeleton_trf.origin = animator_state.skeleton_transform.origin;
	} else {
		if (p_options.skeleton_spring_mode == AgentProceduralAnimOptions::CRITICAL) {
			HBSprings::critical_spring_damper_exact_vector3(
					skeleton_trf.origin,
					animator_state.skeleton_position_spring_velocity,
					animator_state.skeleton_transform.origin,
					p_options.skeleton_position_spring_halflife,
					p_delta);
		} else {
			HBSprings::spring_damper_exact_ratio_vector3(
					skeleton_trf.origin,
					animator_state.skeleton_position_spring_velocity,
					Vector3(),
					animator_state.skeleton_transform.origin,
					p_options.skeleton_position_spring_halflife,
					p_options.skeleton_position_spring_damping_ratio,
					p_delta);
		}
	}
	animator_state.skeleton_output_transform = skeleton_trf;
	HBSprings::critical_spring_damper_exact_vector3(
			animator_state.skeleton_position_offset,
			animator_state.skeleton_position_offset_spring_velocity,
			animator_state.skeleton_position_offset_target,
			p_options.skeleton_position_offset_spring_halflife,
			p_delta);
}

bool AgentProceduralAnimator::have_springs_converged() const {
	for (int i = 0; i < LIMB_MAX; i++) {
		if (animator_state.limb_position_spring_velocities[i].length() > 0.1f) {
			return false;
		}
	}
	return true;
}

void AgentProceduralAnimator::process(AgentProceduralAnimOptions &p_options, float p_delta) {
	animation_time += p_delta;
	animation_time = MIN(animation_time, animation_duration);

	float total_weight = 0.0f;
	float weights[LIMB_MAX];
	Vector3 limb_avg = Vector3();
	int average_limb_count = 0;
	for (int i = 0; i < LIMB_MAX; i++) {
		if (p_options.limb_dangle_status[i]) {
			continue;
		}
		AgentLimb limb = static_cast<AgentLimb>(i);
		float t = CLAMP(get_limb_time(p_options, limb), 0.0, 1.0f);
		animator_state.limb_transforms[i] = p_options.starting_limb_transforms[i].interpolate_with(p_options.target_limb_transforms[i], t);
		animator_state.limb_transforms[i].origin += animator_state.limb_transforms[i].basis.xform(p_options.limb_peak_position[i]) * sin(Math_PI * t) * p_options.anim_blend;
		weights[i] = get_limb_weight(t);
		total_weight += weights[i];
		limb_avg += animator_state.limb_transforms[i].origin;
		average_limb_count++;
	}

	Vector3 limb_weighted_avg;

	for (int i = 0; i < LIMB_MAX; i++) {
		if (p_options.limb_dangle_status[i]) {
			continue;
		}
		limb_weighted_avg += (animator_state.limb_transforms[i].origin) * (weights[i] / total_weight);
	}

	limb_avg /= (float)average_limb_count;

	animator_state.skeleton_position_offset_target = (limb_weighted_avg - limb_avg) * p_options.anim_blend;

	if (p_options.use_average_for_skeleton_trf) {
		animator_state.skeleton_transform.origin = animator_state.limb_transforms[LIMB_LEFT_HAND].origin + animator_state.limb_transforms[LIMB_RIGHT_HAND].origin;
		animator_state.skeleton_transform.origin /= 2.0f;
		Transform3D og_trf = p_options.starting_skeleton_transform.interpolate_with(p_options.target_skeleton_transform, animation_time / animation_duration);
		animator_state.skeleton_transform.basis = og_trf.basis;
	} else {
		animator_state.skeleton_transform = p_options.starting_skeleton_transform.interpolate_with(p_options.target_skeleton_transform, animation_time / animation_duration);
	}

	if (animator_state.restart_queued) {
		animator_state.restart_queued = false;
		print_line("RESTART!");
		for (int i = 0; i < LIMB_MAX; i++) {
			animator_state.limb_output_transforms[i] = animator_state.limb_transforms[i];
		}
		animator_state.skeleton_output_transform = animator_state.skeleton_transform;
	}

	_process_springs(p_options, p_delta);
}

bool AgentProceduralAnimator::is_done() const {
	return animation_time >= animation_duration;
}

void AgentProceduralAnimator::get_output_pose(AgentProceduralPose &p_pose) const {
	p_pose.skeleton_trf = animator_state.skeleton_output_transform;
	p_pose.skeleton_position_offset = animator_state.skeleton_position_offset;
	for (int i = 0; i < LIMB_MAX; i++) {
		p_pose.ik_targets[i] = animator_state.limb_output_transforms[i];
	}
	p_pose.valid = true;
}

void AgentProceduralAnimator::get_unsprung_pose(AgentProceduralPose &p_pose) {
	p_pose.skeleton_trf = animator_state.skeleton_transform;
	for (int i = 0; i < LIMB_MAX; i++) {
		p_pose.ik_targets[i] = animator_state.limb_transforms[i];
	}
	p_pose.valid = true;
}

void AgentProceduralAnimator::reset() {
	animation_time = 0.0f;
}

void AgentProceduralAnimator::seek(float p_playback_position) {
	animation_time = p_playback_position;
}

float AgentProceduralAnimator::get_animation_duration() const { return animation_duration; }

void AgentProceduralAnimator::set_animation_duration(float p_animation_duration) { animation_duration = p_animation_duration; }

AgentProceduralAnimator::AnimatorState AgentProceduralAnimator::get_animator_state() {
	return animator_state;
}

void AgentProceduralAnimator::set_animator_state(const AnimatorState &p_animator_state) {
	animator_state = p_animator_state;
}

void AgentProceduralAnimator::restart() {
	reset();
	animator_state.restart_queued = true;
	for (int i = 0; i < LIMB_MAX; i++) {
		animator_state.limb_position_spring_velocities[i] = Vector3();
		animator_state.limb_rotation_spring_velocities[i] = Vector3();
	}
	animator_state.skeleton_position_spring_velocity = Vector3();
	animator_state.skeleton_rotation_spring_velocity = Vector3();
}

float AgentProceduralAnimator::get_playback_position() const {
	return animation_time;
}

float AgentProceduralAnimator::get_limb_time(const AgentProceduralAnimOptions &p_options, const AgentLimb &p_limb) const {
	float start = p_options.limb_animation_timings[p_limb][0];
	float end = p_options.limb_animation_timings[p_limb][1];
	if (p_options.playback_direction == -1.0f) {
		start = 1.0f - start;
		end = 1.0f - end;

		SWAP(start, end);
	}
	float t = Math::inverse_lerp(start, end, animation_time / animation_duration);
	return t;
}
