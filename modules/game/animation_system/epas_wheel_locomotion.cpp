#include "epas_wheel_locomotion.h"
#include "../resources/game_tools_theme.h"
#include "../springs.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_controller.h"
#include "modules/game/physics_layers.h"
#include "scene/resources/primitive_meshes.h"
#include <iterator>

static bool test_flag = false;

void EPASWheelLocomotion::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	time += p_delta;

	if (!debug_foot_texture.is_valid()) {
		debug_foot_texture = GameToolsTheme::generate_icon(GameToolsThemeIcons::FOOT);
		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
		mat->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA_SCISSOR);
		mat->set_texture(BaseMaterial3D::TEXTURE_ALBEDO, GameToolsTheme::generate_icon(GameToolsThemeIcons::FOOT));

		Ref<QuadMesh> mesh;
		mesh.instantiate();
		mesh->set_size(Vector2(0.2f, 0.2f));
		mesh->set_material(mat);
		mesh->set_orientation(QuadMesh::FACE_Y);
		for (int i = 0; i < MAX_DEBUG_FOOT_MESHES; i++) {
			Decal *mi = memnew(Decal);
			mi->set_cull_mask(0b10);
			mi->set_lower_fade(0.0f);
			mi->set_upper_fade(0.0f);
			mi->set_size(Vector3(0.2f, 0.5f, 0.2f));
			mi->set_as_top_level(true);
			mi->hide();
			mi->set_texture(Decal::TEXTURE_ALBEDO, debug_foot_texture);
			mi->set_modulate(Color("BLUE"));
			debug_foot_meshes.append(mi);
			get_epas_controller()->add_child(mi);
		}
		for (int i = 0; i < 2; i++) {
			Decal *mi = memnew(Decal);
			mi->set_lower_fade(0.0f);
			mi->set_upper_fade(0.0f);
			mi->set_size(Vector3(0.2f, 0.5f, 0.2f));
			mi->set_as_top_level(true);
			mi->show();
			mi->set_texture(Decal::TEXTURE_ALBEDO, debug_foot_texture);
			debug_foot_meshes_predict[i] = mi;
			get_epas_controller()->add_child(mi);
			mi->set_modulate(Color("PURPLE"));
		}
	}

	if (sorted_locomotion_sets.size() == 0) {
		return;
	}

	if (!foot_ik_init && use_foot_ik) {
		debug_geo = memnew(HBDebugGeometry);
		debug_geo->show();
		get_skeleton()->add_child(debug_geo);
		debug_geo->set_as_top_level(true);
		debug_geo->set_global_transform(Transform3D());
		foot_ik_init = true;
	}

	// Janky-ass bsearch to find where we are at right now.
	LocomotionSet set;
	set.x_pos = x_blend;
	int first_greater = sorted_locomotion_sets.bsearch_custom<LocomotionSetComparator>(&set, false);

	float travelled_this_frame = linear_velocity.length() * p_delta;

	if (shared_info.is_valid()) {
		wheel_angle = shared_info->wheel_angle;
	}

	if (sorted_locomotion_sets.size() == 1 || first_greater == 0 || first_greater > locomotion_sets.size()) {
		// Scenario 1, single set, no interpolation

		// Bring index into range
		int i = CLAMP(first_greater, 0, sorted_locomotion_sets.size() - 1);
		const LocomotionSet *first_set = sorted_locomotion_sets[i];

		current_step_length = first_set->step_length;

		float circumference = current_step_length * 4.0f;
		if (circumference > 0.0f) {
			wheel_angle += (travelled_this_frame / circumference) * Math_TAU;
		}
		// 0-1 the cycle of the animation we are in (Contact_L-Contact_L)
		float cycle_time = Math::fmod(wheel_angle, (float)Math_PI) / Math_PI;
		cycle_time = first_set->set_type == LocomotionSetType::WHEEL ? cycle_time : time;

		float foot_ik_grounded[2];
		first_set->interpolate(cycle_time, p_base_pose, p_target_pose, foot_ik_grounded);
	} else {
		// Scenario 2, interpolate between two sets
		// Ensure we are in range
		if (first_greater == sorted_locomotion_sets.size()) {
			first_greater -= 1;
		}
		int first_idx = first_greater - 1;
		ERR_FAIL_COND_MSG(first_greater == 0, "Something catastrophic happened when blending locomotion sets. You might be missing a 0.0 blend time pose.");
		int second_idx = first_greater;
		LocomotionSet *first_set = sorted_locomotion_sets[first_idx];
		LocomotionSet *second_set = sorted_locomotion_sets[second_idx];
		float x = Math::inverse_lerp(first_set->x_pos, second_set->x_pos, x_blend);
		// Calculate the step size, based both step sizes and distance travelled
		current_step_length = Math::lerp(first_set->step_length, second_set->step_length, x);

		// Apply it to the wheel angle
		float circumference = current_step_length * 4.0f;
		float prev_cycle_time = Math::fmod(wheel_angle, (float)Math_PI) / Math_PI;
		if (circumference > 0.0f) {
			wheel_angle += (travelled_this_frame / circumference) * Math_TAU;
		}

		// 0-1 the cycle of the animation we are in (Contact_L-Contact_L)
		float cycle_time = Math::fmod(wheel_angle, (float)Math_PI) / Math_PI;

		// Constant velocity sets use the global time, wheel sets use the current wheel cycle position
		float first_set_cycle_time = first_set->set_type == LocomotionSetType::WHEEL ? cycle_time : Math::fmod(time, first_set->animation->get_length());
		float second_set_cycle_time = second_set->set_type == LocomotionSetType::WHEEL ? cycle_time : Math::fmod(time, second_set->animation->get_length());

		Ref<EPASPose> second_pose;
		second_pose.instantiate();
		float foot_ik_grounded[2];
		float foot_ik_grounded_second[2];
		// We sample both locomotion sets
		first_set->interpolate(first_set_cycle_time, p_base_pose, p_target_pose, foot_ik_grounded);
		second_set->interpolate(second_set_cycle_time, p_base_pose, second_pose, foot_ik_grounded_second);
		Transform3D pre_blend_ankle_trfs[2];

		if (use_foot_ik) {
			for (int i = 0; i < 2; i++) {
				pre_blend_ankle_trfs[i] = p_target_pose->calculate_bone_global_transform(foot_ik[i].bone_name, get_skeleton(), p_base_pose);
			}
		}

		// Then we blend them together based on x

		if (use_foot_ik) {
			LocomotionSet *sets[2] = {
				first_set,
				second_set
			};
			float foot_ik_grounded_amounts[2][2] = {
				{ foot_ik_grounded[0], foot_ik_grounded[1] },
				{ foot_ik_grounded_second[0], foot_ik_grounded_second[1] }
			};
			_ik_process(
					sets,
					foot_ik_grounded_amounts,
					p_target_pose,
					second_pose,
					p_base_pose,
					x,
					p_delta);
		}

		p_target_pose->blend(second_pose, p_base_pose, p_target_pose, x);

		if (!String(root_bone_name).is_empty()) {
			Vector3 pos = p_target_pose->get_bone_position(root_bone_name, p_base_pose);
			if (!p_target_pose->has_bone(root_bone_name)) {
				p_target_pose->create_bone(root_bone_name);
			}
			float current_bounce_height = Math::lerp(first_set->bounce_height, second_set->bounce_height, x);
			// contact_l 0.0 = 0.0
			// pass_l 0.25 	 = 1.0
			// contact_r 0.5 = 0.0
			// pass_r 0.75   = 1.0
			// contact_l 1.0 = 0.0
			float current_bounce = (Math::sin(Math_TAU * 2.0f * cycle_time + 3.0f * Math_PI / 2.0f) + 1.0f) / 2.0f;
			pos.y += current_bounce * current_bounce_height;
			p_target_pose->set_bone_position(root_bone_name, pos);
		}

		// TEST: Process events
		Transform3D foot_trf[2] = {
			foot_ik[0].out_ik_transform,
			foot_ik[1].out_ik_transform,
		};
		LocomotionSet *sets[2] = {
			first_set,
			second_set
		};
		bool prev_lock_states[2];
		bool lock_states[2];
		bool events_processed = false;
		float lock_amount[2];
		float prev_lock_amount[2];
		if (use_foot_ik) {
			events_processed = process_events(
					sets,
					cycle_time,
					x,
					prev_cycle_time,
					prev_lock_states,
					lock_states,
					prev_lock_amount,
					lock_amount);
		}
		if (events_processed) {
			for (int i = 0; i < 2; i++) {
				StringName bone_name = i == 0 ? "foot.L" : "foot.R";
				if ((!foot_ik[i].pinned || !prev_lock_states[i]) && lock_states[i]) {
					// Just pinned, calculate the pinning position
					Transform3D base_trf = p_base_pose->calculate_bone_global_transform(bone_name, get_skeleton(), p_base_pose);

					Vector3 forward = base_trf.basis.get_rotation_quaternion().xform_inv(Vector3(0.0f, 0.0f, 1.0f));
					Vector3 up = base_trf.basis.get_rotation_quaternion().xform_inv(Vector3(0.0f, 1.0f, 0.0f));
					forward = foot_trf[i].basis.get_rotation_quaternion().xform(forward);
					up = foot_trf[i].xform(up);
					// We just locked, draw footsteps
					debug_foot_meshes[curr_debug_foot_mesh]->show();
					Transform3D decal_trf;
					decal_trf.origin = foot_trf[i].origin;
					decal_trf.basis = Quaternion(Vector3(0.0f, 0.0f, -1.0f), forward);

					debug_foot_meshes[curr_debug_foot_mesh]->set_global_transform(decal_trf);

					curr_debug_foot_mesh = (curr_debug_foot_mesh + 1) % MAX_DEBUG_FOOT_MESHES;

					foot_ik[i].pinned_position = foot_trf[i].origin;
					foot_ik[i].pin_recovery_t = 0.0f;
				}
				foot_ik[i].pinned = lock_states[i];
				// Pinned position with a little slide margin applied.
				Vector3 pinned_position_r = foot_ik[i].pinned_position;
				Vector3 target_ik = foot_ik[i].ik_node->get_target_transform().origin;
				target_ik.y = pinned_position_r.y;
				Vector3 pinned_to_ik = target_ik - pinned_position_r;
				pinned_position_r += pinned_to_ik.limit_length(foot_ik_slide_max);

				if (foot_ik[i].pinned) {
					Transform3D target_trf = foot_ik[i].ik_node->get_target_transform();
					target_trf.origin = pinned_position_r;
					foot_ik[i].ik_node->set_target_transform(target_trf);
					debug_geo->debug_sphere(foot_ik[i].pinned_position, 0.01f, Color("RED"));
					debug_geo->debug_sphere(pinned_position_r, 0.01f, Color("GREEN"));
				}

				if (!foot_ik[i].pinned) {
					foot_ik[i].pin_recovery_t = MIN(foot_ik[i].pin_recovery_t + p_delta, foot_ik_pin_recovery_time);
					// When unpinning we interpolate the pinned position to the new target position
					Transform3D target_trf = foot_ik[i].ik_node->get_target_transform();
					target_trf.origin = pinned_position_r.lerp(target_trf.origin, foot_ik[i].pin_recovery_t / foot_ik_pin_recovery_time);
					foot_ik[i].ik_node->set_target_transform(target_trf);
				}

				foot_ik[i].prev_positions[0] = foot_ik[i].prev_positions[1];
				foot_ik[i].prev_positions[1] = foot_ik[i].ik_node->get_target_transform().origin;
			}
		}
	}

	if (shared_info.is_valid()) {
		shared_info->wheel_angle = wheel_angle;
	}
}

void EPASWheelLocomotion::_sort_sets() {
	sorted_locomotion_sets.sort_custom<LocomotionSetComparator>();
}

void EPASWheelLocomotion::_ik_process_foot(LocomotionSet *p_loc_set, float p_foot_ik_grounded[2], Transform3D p_ankle_ik_targets[2], Transform3D p_ankle_pinned_ik_targets[2], const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose) {
	if (!use_foot_ik || !foot_ik_init) {
		return;
	}
	Skeleton3D *skel = get_skeleton();
	PhysicsDirectSpaceState3D *dss = skel->get_world_3d()->get_direct_space_state();
	for (int i = 0; i < 2; i++) {
		StringName ankle_bone_name = foot_ik[i].bone_name;
		Transform3D ankle_global_trf = p_target_pose->calculate_bone_global_transform(ankle_bone_name, skel, p_base_pose);
		ankle_global_trf = skel->get_global_transform() * ankle_global_trf;

		PhysicsDirectSpaceState3D::RayResult result;

		Transform3D hip_trf = p_target_pose->calculate_bone_global_transform(hip_bone_name, skel, p_base_pose);
		Transform3D hip_global_trf = skel->get_global_transform() * hip_trf;
		float ankle_height = p_target_pose->calculate_bone_global_transform(ankle_bone_name, skel, p_base_pose).origin.y;

		PhysicsDirectSpaceState3D::RayParameters params;
		params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;

		params.from = ankle_global_trf.origin;
		params.from.y = hip_global_trf.origin.y;
		params.to = ankle_global_trf.origin;

		p_ankle_ik_targets[i] = ankle_global_trf;
		// This ensures the ankle is always above the ground
		if (dss->intersect_ray(params, result)) {
			p_ankle_ik_targets[i].origin.y = MAX(p_ankle_ik_targets[i].origin.y, result.position.y + ankle_height);
		}

		params.to.y = hip_global_trf.origin.y - hip_trf.origin.y - 0.5f;

		// Intersect to current ankle location
		p_ankle_pinned_ik_targets[i] = ankle_global_trf;
		if (dss->intersect_ray(params, result)) {
			result.position += result.normal * ankle_height;
			p_ankle_pinned_ik_targets[i].origin = result.position;
		} else {
			continue;
		}

		// Rotate the foot using the groud
		Vector<int> ankle_bone_children = get_skeleton()->get_bone_children(get_skeleton()->find_bone(ankle_bone_name));

		ERR_CONTINUE_MSG(ankle_bone_children.size() == 0, "Ankle has no children");

		StringName toe_name = get_skeleton()->get_bone_name(ankle_bone_children[0]);

		Vector3 toe_relative_pos = p_target_pose->get_bone_position(toe_name, p_base_pose);
		Vector3 toe_global_pos = p_ankle_pinned_ik_targets[i].xform(toe_relative_pos);

		Vector3 ankle_to_toe = toe_global_pos - p_ankle_pinned_ik_targets[i].origin;
		ankle_to_toe.y = 0.0f;
		Vector3 model_global_forward = p_ankle_pinned_ik_targets[i].origin.direction_to(toe_global_pos);
		model_global_forward.y = 0.0f;
		model_global_forward.normalize();
		if (!model_global_forward.is_normalized()) {
			model_global_forward = get_skeleton()->get_global_transform().basis.xform(Vector3(0.0f, 0.0f, 1.0f));
		}

		float dist_to_ankle = ankle_to_toe.length();
		params.from += model_global_forward * dist_to_ankle;
		params.to += model_global_forward * dist_to_ankle;

		if (dss->intersect_ray(params, result)) {
			Quaternion new_rot = Quaternion(Vector3(0.0f, 1.0f, 0.0f), result.normal) * p_ankle_pinned_ik_targets[i].basis;
			p_ankle_pinned_ik_targets[i].basis = new_rot;
		}
	}
}

bool find_thigh_and_knee_from_ankle(StringName p_ankle_bone_name, StringName *p_thigh_bone_name, StringName *p_knee_bone_name, Skeleton3D *p_skeleton) {
	int ankle_bone_idx = p_skeleton->find_bone(p_ankle_bone_name);
	ERR_FAIL_COND_V_MSG(ankle_bone_idx == -1, false, vformat("Couldnt find ankle bone %s in skeleton", p_ankle_bone_name));
	int knee_bone_idx = p_skeleton->get_bone_parent(ankle_bone_idx);
	ERR_FAIL_COND_V_MSG(knee_bone_idx == -1, false, vformat("Couldnt find knee bone for ankle bone %s in skeleton", p_ankle_bone_name));
	int thigh_bone_idx = p_skeleton->get_bone_parent(knee_bone_idx);
	ERR_FAIL_COND_V_MSG(thigh_bone_idx == -1, false, vformat("Couldnt find thigh bone for ankle bone %s in skeleton", p_ankle_bone_name));

	*p_thigh_bone_name = p_skeleton->get_bone_name(thigh_bone_idx);
	*p_knee_bone_name = p_skeleton->get_bone_name(knee_bone_idx);
	return true;
}

void calculate_ik_pole_position(StringName p_ankle_bone_name, Vector3 ik_target, Ref<EPASPose> p_pose, Ref<EPASPose> p_base_pose, Vector3 *p_new_pole, Skeleton3D *p_skeleton) {
	StringName thigh_bone_name;
	StringName knee_bone_name;

	find_thigh_and_knee_from_ankle(p_ankle_bone_name, &thigh_bone_name, &knee_bone_name, p_skeleton);

	Transform3D thigh_trf = p_skeleton->get_global_transform() * p_pose->calculate_bone_global_transform(thigh_bone_name, p_skeleton, p_base_pose);
	Transform3D knee_trf = p_skeleton->get_global_transform() * p_pose->calculate_bone_global_transform(knee_bone_name, p_skeleton, p_base_pose);
	Transform3D ankle_trf = p_skeleton->get_global_transform() * p_pose->calculate_bone_global_transform(p_ankle_bone_name, p_skeleton, p_base_pose);

	Vector3 ankle_to_thigh = thigh_trf.origin - ankle_trf.origin;
	Vector3 ankle_to_knee = knee_trf.origin - ankle_trf.origin;
	float ankle_to_thigh_percentage = ankle_to_knee.project(ankle_to_knee).length() / ankle_to_thigh.length();
	Vector3 knee_forward = (ankle_to_thigh * ankle_to_thigh_percentage).direction_to(ankle_to_knee);

	*p_new_pole = p_skeleton->get_global_position() + knee_forward * 2.0f;
}

void EPASWheelLocomotion::_ik_process(LocomotionSet *p_loc_sets[2], float p_foot_ik_grounded[2][2], Ref<EPASPose> p_target_pose, Ref<EPASPose> p_second_pose, Ref<EPASPose> p_base_pose, float p_x, float p_delta) {
	Transform3D ankle_global_ik_targets[2];
	Transform3D ankle_global_ik_targets_second[2];
	Transform3D ankle_global_ik_pinned_targets[2];
	Transform3D ankle_global_ik_pinned_targets_second[2];
	debug_geo->clear();
	test_flag = true;
	_ik_process_foot(p_loc_sets[0], p_foot_ik_grounded[0], ankle_global_ik_targets, ankle_global_ik_pinned_targets, p_base_pose, p_target_pose);
	test_flag = false;
	_ik_process_foot(p_loc_sets[0], p_foot_ik_grounded[1], ankle_global_ik_targets_second, ankle_global_ik_pinned_targets_second, p_base_pose, p_second_pose);

	for (int i = 0; i < 2; i++) {
		Transform3D first_ik_target = ankle_global_ik_targets[i].interpolate_with(ankle_global_ik_pinned_targets[i], p_foot_ik_grounded[0][i]);
		Transform3D second_ik_target = ankle_global_ik_targets_second[i].interpolate_with(ankle_global_ik_pinned_targets_second[i], p_foot_ik_grounded[1][i]);

		foot_ik[i].out_ik_transform = first_ik_target.interpolate_with(second_ik_target, p_x);
		DEV_ASSERT(foot_ik[i].ik_node.is_valid());
		foot_ik[i].ik_node->set_target_transform(foot_ik[i].out_ik_transform);

		Transform3D first_original_ankle = get_skeleton()->get_global_transform() * p_target_pose->calculate_bone_global_transform(foot_ik[i].bone_name, get_skeleton(), p_base_pose);
		Transform3D second_original_ankle = get_skeleton()->get_global_transform() * p_second_pose->calculate_bone_global_transform(foot_ik[i].bone_name, get_skeleton(), p_base_pose);

		Vector3 pole_position_first;
		Vector3 pole_position_second;
		calculate_ik_pole_position(foot_ik[i].bone_name, first_ik_target.origin, p_target_pose, p_base_pose, &pole_position_first, get_skeleton());
		calculate_ik_pole_position(foot_ik[i].bone_name, second_ik_target.origin, p_second_pose, p_base_pose, &pole_position_second, get_skeleton());

		foot_ik[i].out_ik_magnet_position = pole_position_first.lerp(pole_position_second, p_x);
		foot_ik[i].ik_node->set_magnet_position(foot_ik[i].out_ik_magnet_position);

		// Calculate new pole position
		// to do this, we grab both original bones and use the to figure a percentage from thigh to ankle to use as the starting point
		// then we use the original knee orientation as a forward vector and place it accordingly

		// Calculate and apply hip offset to poses
		Vector3 offset_first;
		offset_first.y = (first_ik_target.origin.y - first_original_ankle.origin.y);
		Vector3 offset_second;
		offset_second.y = (second_ik_target.origin.y - second_original_ankle.origin.y);

		if (!p_target_pose->has_bone(hip_bone_name)) {
			p_target_pose->create_bone(hip_bone_name);
		}

		if (!p_second_pose->has_bone(hip_bone_name)) {
			p_second_pose->create_bone(hip_bone_name);
		}

		// Adjust hip position
		// LAZY HACK: Assuming root is 0,0,0 and hip is a child of it...
		float hl = 0.05f;
		Vector3 new_first_bone_pos = p_target_pose->get_bone_position(hip_bone_name, p_base_pose);
		HBSprings::critical_spring_damper_exact_vector3(p_loc_sets[0]->hip_offset, p_loc_sets[0]->hip_offset_spring_vel, offset_first, hl, p_delta);
		new_first_bone_pos += p_loc_sets[0]->hip_offset;
		p_target_pose->set_bone_position(hip_bone_name, new_first_bone_pos);

		Vector3 new_second_bone_pos = p_second_pose->get_bone_position(hip_bone_name, p_base_pose);
		HBSprings::critical_spring_damper_exact_vector3(p_loc_sets[1]->hip_offset, p_loc_sets[1]->hip_offset_spring_vel, offset_second, hl, p_delta);
		new_second_bone_pos += p_loc_sets[1]->hip_offset;
		p_second_pose->set_bone_position(hip_bone_name, new_second_bone_pos);
	}
}

float EPASWheelLocomotion::find_next_feet_ground_time(Ref<EPASAnimation> p_anim, float p_times[2]) const {
}

bool EPASWheelLocomotion::process_events(LocomotionSet *p_sets[2], float p_time, float p_blend, float p_previous_time, bool p_out_prev_lock_state[2], bool p_out_lock_state[2], float p_out_prev_lock_amount[2], float p_out_lock_amount[2]) {
	Ref<Curve> curve_left_first = p_sets[0]->animation->get_animation_curve("left_ik_grounded");
	Ref<Curve> curve_right_first = p_sets[0]->animation->get_animation_curve("right_ik_grounded");
	Ref<Curve> curve_left_second = p_sets[1]->animation->get_animation_curve("left_ik_grounded");
	Ref<Curve> curve_right_second = p_sets[1]->animation->get_animation_curve("right_ik_grounded");

	Ref<Curve> curves[4] = {
		curve_left_first,
		curve_left_second,
		curve_right_first,
		curve_right_second
	};

	for (int i = 0; i < 4; i++) {
		if (!curves[i].is_valid()) {
			return false;
		}
	}

	for (int i = 0; i < 2; i++) {
		int first = i * 2;
		int second = first + 1;

		float curve_sample_prev_first = curves[first]->sample(p_previous_time);
		float curve_sample_prev_second = curves[second]->sample(p_previous_time);
		float curve_sample_first = curves[first]->sample(p_time);
		float curve_sample_second = curves[second]->sample(p_time);

		float curve_sample_prev = Math::lerp(curve_sample_prev_first, curve_sample_prev_second, p_blend);
		float curve_sample = Math::lerp(curve_sample_first, curve_sample_second, p_blend);
		curve_sample_prev = MIN(curve_sample_prev, 1.0f);
		curve_sample = MIN(curve_sample, 1.0f);

		p_out_prev_lock_state[i] = Math::is_equal_approx(curve_sample_prev, 1.0f);
		p_out_lock_state[i] = Math::is_equal_approx(curve_sample, 1.0f);

		p_out_prev_lock_amount[i] = curve_sample_prev;
		p_out_lock_amount[i] = curve_sample;
	}

	return true;
}

int EPASWheelLocomotion::get_locomotion_set_count() const {
	return locomotion_sets.size();
}

void EPASWheelLocomotion::add_locomotion_set(float p_x) {
	ERR_FAIL_COND(p_x > 1.0);
	ERR_FAIL_COND(p_x < 0.0);
	LocomotionSet *set = memnew(LocomotionSet);
	set->x_pos = p_x;
	locomotion_sets.push_back(set);
	sorted_locomotion_sets.push_back(set);
	_sort_sets();
}

void EPASWheelLocomotion::set_locomotion_set_step_length(int p_idx, float p_step) {
	ERR_FAIL_INDEX_MSG(p_idx, locomotion_sets.size(), "Locomotion set out of range");
	ERR_FAIL_COND_MSG(p_step < 0.0, "Step length cannot be under 0.0");
	locomotion_sets[p_idx]->step_length = p_step;
}

void EPASWheelLocomotion::set_locomotion_set_animation(int p_idx, Ref<EPASAnimation> p_animation) {
	ERR_FAIL_INDEX_MSG(p_idx, locomotion_sets.size(), "Locomotion set out of range");
	locomotion_sets[p_idx]->animation = p_animation;
}

void EPASWheelLocomotion::set_locomotion_set_type(int p_idx, LocomotionSetType p_type) {
	ERR_FAIL_INDEX_MSG(p_idx, locomotion_sets.size(), "Locomotion set out of range");
	locomotion_sets[p_idx]->set_type = p_type;
}

void EPASWheelLocomotion::set_locomotion_set_bounce_height(int p_idx, float p_bounce_height) {
	ERR_FAIL_INDEX_MSG(p_idx, locomotion_sets.size(), "Locomotion set out of range");
	locomotion_sets[p_idx]->bounce_height = p_bounce_height;
}

StringName EPASWheelLocomotion::get_root_bone_name() const {
	return root_bone_name;
}

void EPASWheelLocomotion::set_root_bone_name(const StringName &p_root_bone_name) {
	root_bone_name = p_root_bone_name;
}

bool EPASWheelLocomotion::get_use_foot_ik() const {
	return use_foot_ik;
}

void EPASWheelLocomotion::set_use_foot_ik(bool p_use_foot_ik) {
	use_foot_ik = p_use_foot_ik;
}

void EPASWheelLocomotion::set_left_foot_ik_node(Ref<EPASIKNode> p_ik_left_foot_ik_node) {
	foot_ik[0].ik_node = p_ik_left_foot_ik_node;
}

void EPASWheelLocomotion::set_right_foot_ik_node(Ref<EPASIKNode> p_ik_right_foot_ik_node) {
	foot_ik[1].ik_node = p_ik_right_foot_ik_node;
}

StringName EPASWheelLocomotion::get_left_foot_bone_name() const {
	return foot_ik[0].bone_name;
}

void EPASWheelLocomotion::set_left_foot_bone_name(const StringName &p_left_foot_bone_name) {
	foot_ik[0].bone_name = p_left_foot_bone_name;
}

StringName EPASWheelLocomotion::get_right_foot_bone_name() const {
	return foot_ik[1].bone_name;
}

void EPASWheelLocomotion::set_right_foot_bone_name(const StringName &p_right_foot_bone_name) {
	foot_ik[1].bone_name = p_right_foot_bone_name;
}

float EPASWheelLocomotion::get_wheel_angle() const {
	return wheel_angle;
}

void EPASWheelLocomotion::set_linear_velocity(const Vector3 &p_linear_velocity) {
	linear_velocity = p_linear_velocity;
}

void EPASWheelLocomotion::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_linear_velocity", "velocity"), &EPASWheelLocomotion::set_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_wheel_angle"), &EPASWheelLocomotion::get_wheel_angle);
	ClassDB::bind_method(D_METHOD("get_current_step_length"), &EPASWheelLocomotion::get_current_step_length);
	ClassDB::bind_method(D_METHOD("set_x_blend", "x"), &EPASWheelLocomotion::set_x_blend);
	ClassDB::bind_method(D_METHOD("get_x_blend"), &EPASWheelLocomotion::get_x_blend);
	ClassDB::bind_method(D_METHOD("add_locomotion_set", "x"), &EPASWheelLocomotion::add_locomotion_set);
	ClassDB::bind_method(D_METHOD("get_locomotion_set_count"), &EPASWheelLocomotion::get_locomotion_set_count);
	ClassDB::bind_method(D_METHOD("set_locomotion_set_animation", "idx", "animation"), &EPASWheelLocomotion::set_locomotion_set_animation);
	ClassDB::bind_method(D_METHOD("set_locomotion_set_step_length", "idx", "step_length"), &EPASWheelLocomotion::set_locomotion_set_step_length);
	ClassDB::bind_method(D_METHOD("set_locomotion_set_type", "idx", "type"), &EPASWheelLocomotion::set_locomotion_set_type);
	ClassDB::bind_method(D_METHOD("set_locomotion_set_bounce_height", "idx", "bounce_height"), &EPASWheelLocomotion::set_locomotion_set_bounce_height);
	ClassDB::bind_method(D_METHOD("set_root_bone_name", "root_bone_name"), &EPASWheelLocomotion::set_root_bone_name);
	ClassDB::bind_method(D_METHOD("get_root_bone_name"), &EPASWheelLocomotion::get_root_bone_name);
	ClassDB::bind_method(D_METHOD("set_left_foot_bone_name", "left_foot_bone_name"), &EPASWheelLocomotion::set_left_foot_bone_name);
	ClassDB::bind_method(D_METHOD("get_left_foot_bone_name"), &EPASWheelLocomotion::get_left_foot_bone_name);
	ClassDB::bind_method(D_METHOD("set_right_foot_bone_name", "right_foot_bone_name"), &EPASWheelLocomotion::set_right_foot_bone_name);
	ClassDB::bind_method(D_METHOD("get_right_foot_bone_name"), &EPASWheelLocomotion::get_right_foot_bone_name);
	ClassDB::bind_method(D_METHOD("set_use_foot_ik", "use_foot_ik"), &EPASWheelLocomotion::set_use_foot_ik);
	ClassDB::bind_method(D_METHOD("get_use_foot_ik"), &EPASWheelLocomotion::get_use_foot_ik);
	ClassDB::bind_method(D_METHOD("set_hip_bone_name", "hip_bone_name"), &EPASWheelLocomotion::set_hip_bone_name);
	ClassDB::bind_method(D_METHOD("get_hip_bone_name"), &EPASWheelLocomotion::get_hip_bone_name);
	ClassDB::bind_method(D_METHOD("set_right_foot_ik_node", "right_foot_ik_node"), &EPASWheelLocomotion::set_right_foot_ik_node);
	ClassDB::bind_method(D_METHOD("set_left_foot_ik_node", "left_foot_ik_node"), &EPASWheelLocomotion::set_left_foot_ik_node);
	ClassDB::bind_method(D_METHOD("sync_with", "other_wheel_locomotions"), &EPASWheelLocomotion::sync_with);

	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "left_foot_bone_name"), "set_left_foot_bone_name", "get_left_foot_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "right_foot_bone_name"), "set_right_foot_bone_name", "get_right_foot_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "hip_bone_name"), "set_hip_bone_name", "get_hip_bone_name");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_foot_ik"), "set_use_foot_ik", "get_use_foot_ik");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "x_blend", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_x_blend", "get_x_blend");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "root_bone_name"), "set_root_bone_name", "get_root_bone_name");

	BIND_ENUM_CONSTANT(WHEEL);
	BIND_ENUM_CONSTANT(CONSTANT_VELOCITY);
}

float EPASWheelLocomotion::get_current_step_length() const {
	return current_step_length;
}

void EPASWheelLocomotion::set_x_blend(float p_x_blend) {
	ERR_FAIL_COND(p_x_blend < 0.0);
	ERR_FAIL_COND(p_x_blend > 1.0);
	x_blend = p_x_blend;
}

float EPASWheelLocomotion::get_x_blend() const {
	return x_blend;
}

EPASWheelLocomotion::~EPASWheelLocomotion() {
	for (int i = 0; i < locomotion_sets.size(); i++) {
		memdelete(locomotion_sets[i]);
	}
	locomotion_sets.clear();
	sorted_locomotion_sets.clear();
}

StringName EPASWheelLocomotion::get_hip_bone_name() const { return hip_bone_name; }

void EPASWheelLocomotion::set_hip_bone_name(const StringName &p_hip_bone_name) { hip_bone_name = p_hip_bone_name; }

void EPASWheelLocomotion::sync_with(TypedArray<EPASWheelLocomotion> p_locomotions) {
	if (!shared_info.is_valid()) {
		shared_info.instantiate();
	}
	for (int i = 0; i < p_locomotions.size(); i++) {
		Ref<EPASWheelLocomotion> other_loc = p_locomotions[i];
		other_loc->shared_info = shared_info;
	}
}

void EPASWheelLocomotion::reset_foot_ik() {
	for (size_t i = 0; i < std::size(foot_ik); i++) {
		foot_ik[i].pinned = false;
	}
}

EPASWheelLocomotion::EPASWheelLocomotion() {
}

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
void EPASWheelLocomotion::_debug_node_draw() const {
	ImGui::Text("Wheel angle: %.2f", Math::fmod(Math::rad_to_deg(get_wheel_angle()), 360.0f));
	ImGui::Text("X blend: %.2f", x_blend);
}
#endif