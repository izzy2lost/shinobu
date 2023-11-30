#include "agent_procedural_anim.h"
#include "springs.h"

void AgentProceduralAnimator::_advance_animation(AgentProceduralAnimOptions &p_options, float p_delta) {
	_process_springs(p_options, p_delta);
}

void AgentProceduralAnimator::_process_springs(AgentProceduralAnimOptions &p_options, float p_delta) {
	// Process limb springs
	for (int i = 0; i < LIMB_MAX; i++) {
		Transform3D trf = limb_output_transforms[i];
		float position_spring_halflife = p_options.limb_position_spring_halflifes[i];
		if (position_spring_halflife <= 0.0f) {
			trf.origin = limb_transforms[i].origin;
		} else {
			HBSprings::critical_spring_damper_exact_vector3(
					trf.origin,
					limb_position_spring_velocities[i],
					limb_transforms[i].origin,
					position_spring_halflife,
					p_delta);
		}

		float rotation_spring_halflife = p_options.limb_rotation_spring_halflifes[i];

		if (rotation_spring_halflife <= 0.0f) {
			trf.basis = limb_transforms[i].basis;
		} else {
			Quaternion rot = trf.basis.get_rotation_quaternion();
			HBSprings::simple_spring_damper_exact_quat(
					rot,
					limb_rotation_spring_velocities[i],
					limb_transforms[i].basis.get_rotation_quaternion(),
					rotation_spring_halflife,
					p_delta);
			trf.basis = rot;
		}

		limb_output_transforms[i] = trf;
	}

	// Process skeleton springs
	Transform3D skeleton_trf = skeleton_output_transform;
	if (p_options.skeleton_rotation_spring_halflife <= 0.0f) {
		skeleton_trf.basis = skeleton_transform.basis;
	} else {
		Quaternion rot = skeleton_output_transform.basis.get_rotation_quaternion();
		HBSprings::simple_spring_damper_exact_quat(
				rot,
				skeleton_rotation_spring_velocity,
				skeleton_transform.basis.get_rotation_quaternion(),
				p_options.skeleton_rotation_spring_halflife,
				p_delta);
		skeleton_trf.basis = rot;
	}

	if (p_options.skeleton_position_spring_halflife <= 0.0f) {
		skeleton_trf.origin = skeleton_transform.origin;
	} else {
		HBSprings::critical_spring_damper_exact_vector3(
				skeleton_trf.origin,
				skeleton_position_spring_velocity,
				skeleton_transform.origin,
				p_options.skeleton_position_spring_halflife,
				p_delta);
	}
	skeleton_output_transform = skeleton_trf;
	HBSprings::critical_spring_damper_exact_vector3(
			skeleton_position_offset,
			skeleton_position_offset_spring_velocity,
			skeleton_position_offset_target,
			p_options.skeleton_position_offset_spring_halflife,
			p_delta);
}

void AgentProceduralAnimator::process(AgentProceduralAnimOptions &p_options, float p_delta) {
	animation_time += p_delta;
	animation_time = MIN(animation_time, 1.0f);

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
		limb_transforms[i] = p_options.starting_limb_transforms[i].interpolate_with(p_options.target_limb_transforms[i], t);
		limb_transforms[i].origin += p_options.limb_peak_position[i] * sin(Math_PI * t) * p_options.anim_blend;
		weights[i] = get_limb_weight(t);
		total_weight += weights[i];
		limb_avg += limb_transforms[i].origin;
		average_limb_count++;
	}

	Vector3 limb_weighted_avg;

	for (int i = 0; i < LIMB_MAX; i++) {
		if (p_options.limb_dangle_status[i]) {
			continue;
		}
		limb_weighted_avg += (limb_transforms[i].origin) * (weights[i] / total_weight);
	}

	limb_avg /= average_limb_count;

	skeleton_position_offset_target = (limb_weighted_avg - limb_avg) * p_options.anim_blend;

	skeleton_transform = p_options.starting_skeleton_transform.interpolate_with(p_options.target_skeleton_transform, animation_time);

	skeleton_transform.origin = limb_avg;

	if (restart_queued) {
		restart_queued = false;
		for (int i = 0; i < LIMB_MAX; i++) {
			limb_output_transforms[i] = limb_transforms[i];
		}
		skeleton_output_transform = skeleton_transform;
	}

	_process_springs(p_options, p_delta);
}

bool AgentProceduralAnimator::is_done() const {
	return animation_time >= 1.0f;
}

void AgentProceduralAnimator::get_output_pose(AgentProceduralPose &p_pose) const {
	p_pose.skeleton_trf = skeleton_output_transform;
	p_pose.skeleton_position_offset = skeleton_position_offset;
	for (int i = 0; i < LIMB_MAX; i++) {
		p_pose.ik_targets[i] = limb_output_transforms[i];
	}
	p_pose.valid = true;
}

void AgentProceduralAnimator::get_unsprung_pose(AgentProceduralPose &p_pose) {
	p_pose.skeleton_trf = skeleton_transform;
	for (int i = 0; i < LIMB_MAX; i++) {
		//p_pose.ik_targets[i] = skeleton_transform[i];
	}
	p_pose.valid = true;
}

void AgentProceduralAnimator::reset() {
	animation_time = 0.0f;
}

void AgentProceduralAnimator::seek(float p_playback_position) {
	animation_time = p_playback_position;
}

void AgentProceduralAnimator::restart() {
	reset();
	restart_queued = true;
	for (int i = 0; i < LIMB_MAX; i++) {
		limb_position_spring_velocities[i] = Vector3();
		limb_rotation_spring_velocities[i] = Vector3();
	}
	skeleton_position_spring_velocity = Vector3();
	skeleton_rotation_spring_velocity = Vector3();
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
	float t = Math::inverse_lerp(start, end, animation_time);
	return t;
}
