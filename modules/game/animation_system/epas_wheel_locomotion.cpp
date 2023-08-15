#include "epas_wheel_locomotion.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/physics_layers.h"

static bool test_flag = false;

void EPASWheelLocomotion::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	time += p_delta;
	if (sorted_locomotion_sets.size() == 0) {
		return;
	}

	if (!foot_ik_init && use_foot_ik) {
		debug_geo = memnew(HBDebugGeometry);
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
					x);
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
		if (use_foot_ik) {
			Color red = Color("RED");
			Color green = Color("GREEN");
			for (int i = 0; i < 2; i++) {
				float grounded = Math::lerp(foot_ik_grounded[i], foot_ik_grounded_second[i], x);
				debug_geo->debug_sphere(foot_ik[i].out_ik_transform.origin, 0.05f, red.lerp(green, grounded));
			}
		}
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
		debug_geo->debug_raycast(params);
		if (dss->intersect_ray(params, result)) {
			p_ankle_ik_targets[i].origin.y = MAX(p_ankle_ik_targets[i].origin.y, result.position.y + ankle_height);
		}

		// Intersect to current ankle location
		params.to.y = hip_global_trf.origin.y - hip_trf.origin.y - 0.5f;

		debug_geo->debug_raycast(params, Color("GREEN"));
		if (dss->intersect_ray(params, result)) {
			p_ankle_pinned_ik_targets[i] = ankle_global_trf;
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

		debug_geo->debug_raycast(params, Color("RED"));
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

void EPASWheelLocomotion::_ik_process(LocomotionSet *p_loc_sets[2], float p_foot_ik_grounded[2][2], Ref<EPASPose> p_target_pose, Ref<EPASPose> p_second_pose, Ref<EPASPose> p_base_pose, float p_x) {
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

		Transform3D first_original_ankle = get_skeleton()->get_global_transform() * p_target_pose->calculate_bone_global_transform(foot_ik[i].bone_name, get_skeleton(), p_base_pose);
		Transform3D second_original_ankle = get_skeleton()->get_global_transform() * p_second_pose->calculate_bone_global_transform(foot_ik[i].bone_name, get_skeleton(), p_base_pose);

		Vector3 pole_position_first;
		Vector3 pole_position_second;
		calculate_ik_pole_position(foot_ik[i].bone_name, first_ik_target.origin, p_target_pose, p_base_pose, &pole_position_first, get_skeleton());
		calculate_ik_pole_position(foot_ik[i].bone_name, second_ik_target.origin, p_second_pose, p_base_pose, &pole_position_second, get_skeleton());

		foot_ik[i].out_ik_magnet_position = pole_position_first.lerp(pole_position_second, p_x);

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
		Vector3 new_first_bone_pos = p_target_pose->get_bone_position(hip_bone_name, p_base_pose);
		new_first_bone_pos.y += MIN(offset_first.y, 0.0f);
		new_first_bone_pos.x += offset_first.x;
		new_first_bone_pos.z += offset_first.z;
		p_target_pose->set_bone_position(hip_bone_name, new_first_bone_pos);
		Vector3 new_second_bone_pos = p_second_pose->get_bone_position(hip_bone_name, p_base_pose);
		new_second_bone_pos.y += MIN(offset_second.y, 0.0f);
		new_second_bone_pos.x += offset_second.x;
		new_second_bone_pos.z += offset_second.z;
		p_second_pose->set_bone_position(hip_bone_name, new_second_bone_pos);
	}
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

Transform3D EPASWheelLocomotion::get_left_foot_ik_target() const {
	return foot_ik[0].out_ik_transform;
}

Transform3D EPASWheelLocomotion::get_right_foot_ik_target() const {
	return foot_ik[1].out_ik_transform;
}

Vector3 EPASWheelLocomotion::get_left_foot_ik_magnet() const {
	return foot_ik[0].out_ik_magnet_position;
}

Vector3 EPASWheelLocomotion::get_right_foot_ik_magnet() const {
	return foot_ik[1].out_ik_magnet_position;
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
	ClassDB::bind_method(D_METHOD("get_left_foot_ik_target"), &EPASWheelLocomotion::get_left_foot_ik_target);
	ClassDB::bind_method(D_METHOD("get_right_foot_ik_target"), &EPASWheelLocomotion::get_right_foot_ik_target);
	ClassDB::bind_method(D_METHOD("set_use_foot_ik", "use_foot_ik"), &EPASWheelLocomotion::set_use_foot_ik);
	ClassDB::bind_method(D_METHOD("get_use_foot_ik"), &EPASWheelLocomotion::get_use_foot_ik);
	ClassDB::bind_method(D_METHOD("set_hip_bone_name", "hip_bone_name"), &EPASWheelLocomotion::set_hip_bone_name);
	ClassDB::bind_method(D_METHOD("get_hip_bone_name"), &EPASWheelLocomotion::get_hip_bone_name);
	ClassDB::bind_method(D_METHOD("get_left_foot_ik_magnet"), &EPASWheelLocomotion::get_left_foot_ik_magnet);
	ClassDB::bind_method(D_METHOD("get_right_foot_ik_magnet"), &EPASWheelLocomotion::get_right_foot_ik_magnet);

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