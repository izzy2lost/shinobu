#include "epas_wheel_locomotion.h"
#include "modules/game/animation_system/epas_animation.h"

void EPASWheelLocomotion::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	time += p_delta;
	if (sorted_locomotion_sets.size() > 0) {
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

			first_set->interpolate(cycle_time, p_base_pose, p_target_pose);
		} else {
			// Scenario 2, interpolate between two sets
			// Ensure we are in range
			if (first_greater == sorted_locomotion_sets.size()) {
				first_greater -= 1;
			}
			int first_idx = first_greater - 1;
			ERR_FAIL_COND_MSG(first_greater == 0, "Something catastrophic happened when blending locomotion sets. You might be missing a 0.0 blend time pose.");
			int second_idx = first_greater;
			const LocomotionSet *first_set = sorted_locomotion_sets[first_idx];
			const LocomotionSet *second_set = sorted_locomotion_sets[second_idx];
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

			// We sample both locomotion sets
			first_set->interpolate(first_set_cycle_time, p_base_pose, p_target_pose);
			second_set->interpolate(second_set_cycle_time, p_base_pose, second_pose);

			// Then we blend them together based on x_blend
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
		}
	}
}

void EPASWheelLocomotion::_sort_sets() {
	sorted_locomotion_sets.sort_custom<LocomotionSetComparator>();
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

#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
void EPASWheelLocomotion::_debug_node_draw() const {
	ImGui::Text("Wheel angle: %.2f", Math::fmod(Math::rad_to_deg(get_wheel_angle()), 360.0f));
	ImGui::Text("X blend: %.2f", x_blend);
}
#endif