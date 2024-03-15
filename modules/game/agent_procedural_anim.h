/**************************************************************************/
/*  agent_procedural_anim.h                                               */
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

#ifndef AGENT_PROCEDURAL_ANIM_H
#define AGENT_PROCEDURAL_ANIM_H

#include "core/math/transform_3d.h"
#include "core/string/string_name.h"

class AgentProceduralAnimator {
public:
	enum AgentLimb {
		LIMB_LEFT_HAND,
		LIMB_RIGHT_HAND,
		LIMB_LEFT_FOOT,
		LIMB_RIGHT_FOOT,
		LIMB_MAX
	};
	struct AgentProceduralPose {
		Transform3D ik_targets[LIMB_MAX];
		Vector3 ik_magnet_positions[LIMB_MAX];
		Transform3D skeleton_trf;
		Vector3 skeleton_position_offset;
		bool valid = false;
	};

	struct AgentProceduralAnimOptions {
		float anim_blend = 1.0f;
		float limb_position_spring_halflifes[LIMB_MAX] = { 0.250f };
		float limb_rotation_spring_halflifes[LIMB_MAX] = { 0.250f };
		Vector3 limb_peak_position[LIMB_MAX];
		float skeleton_position_spring_halflife = 0.05f;
		float skeleton_position_spring_damping_ratio = 0.8f;
		float skeleton_rotation_spring_halflife = 0.25f;
		enum SkeletonSpringMode {
			CRITICAL,
			DAMPED
		};
		SkeletonSpringMode skeleton_spring_mode = SkeletonSpringMode::CRITICAL;
		float skeleton_position_offset_spring_halflife = 0.25f;
		float limb_animation_timings[LIMB_MAX][2] = { { 0.0f } };
		bool limb_dangle_status[LIMB_MAX] = { false };

		Transform3D starting_limb_transforms[LIMB_MAX];
		Transform3D target_limb_transforms[LIMB_MAX];
		Transform3D starting_skeleton_transform;
		Transform3D target_skeleton_transform;

		bool use_average_for_skeleton_trf = false;

		float playback_direction = 1.0f;
	};
	struct AnimatorState {
		// Unsprung transform, this is what's actually animated by us
		Transform3D limb_transforms[LIMB_MAX] = {};
		// (Potentially) sprung transform
		Vector3 limb_position_spring_velocities[LIMB_MAX] = {};
		Vector3 limb_rotation_spring_velocities[LIMB_MAX] = {};
		Transform3D limb_output_transforms[LIMB_MAX] = {};

		// Unsprung skeleton transform
		Transform3D skeleton_transform;
		Vector3 skeleton_position_offset;
		Vector3 skeleton_position_offset_target;
		Vector3 skeleton_position_spring_velocity;
		Vector3 skeleton_position_offset_spring_velocity;
		Vector3 skeleton_rotation_spring_velocity;
		// Sprung skeleton transform
		Transform3D skeleton_output_transform;
		// This means we should reset the sprung positions next time around
		bool restart_queued = true;
	};

private:
	void _advance_animation(AgentProceduralAnimOptions &p_options, float p_delta);
	void _process_springs(AgentProceduralAnimOptions &p_options, float p_delta);
	float animation_time = 0.0f;
	float animation_duration = 1.0f;
	AnimatorState animator_state;
	static float get_limb_weight(float p_time) {
		if (p_time <= 0.0f || p_time >= 1.0f) {
			return 1.0f;
		}
		float w = 1.0f - sin(p_time * Math_PI);
		return w * w;
	}

public:
	bool have_springs_converged() const;
	void process(AgentProceduralAnimOptions &p_options, float p_delta);
	bool is_done() const;
	void get_output_pose(AgentProceduralPose &p_pose) const;
	void get_unsprung_pose(AgentProceduralPose &p_pose);
	void set_pose(const AgentProceduralPose &p_pose);
	float get_limb_time(const AgentProceduralAnimOptions &p_options, const AgentLimb &p_limb) const;
	float get_playback_position() const;
	// Restarts animation playback from 0 and resets all springs
	void restart();
	// Restarts animation playback from 0
	void reset();
	void seek(float p_playback_position);

	static StringName limb_to_bone_name(AgentLimb p_limb) {
		StringName sn;
		switch (p_limb) {
			case LIMB_LEFT_HAND: {
				sn = "hand.L";
			} break;
			case LIMB_RIGHT_HAND: {
				sn = "hand.R";
			} break;
			case LIMB_LEFT_FOOT: {
				sn = "foot.L";
			} break;
			case LIMB_RIGHT_FOOT: {
				sn = "foot.R";
			} break;
			default: {
				DEV_ASSERT(false);
			} break;
		}
		return sn;
	}

	float get_animation_duration() const;
	void set_animation_duration(float p_animation_duration);
	AnimatorState get_animator_state();
	void set_animator_state(const AnimatorState &p_animator_state);
};

#endif // AGENT_PROCEDURAL_ANIM_H
