/**************************************************************************/
/*  epas_softness_node.cpp                                                */
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

#include "epas_softness_node.h"
#include "../springs.h"
#ifdef DEBUG_ENABLED
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#endif

void EPASSoftnessNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_bone_softness", "bone_name", "softness"), &EPASSoftnessNode::set_bone_softness);
	ClassDB::bind_method(D_METHOD("set_influence", "influence"), &EPASSoftnessNode::set_influence);
	ClassDB::bind_method(D_METHOD("get_influence"), &EPASSoftnessNode::get_influence);
	ClassDB::bind_method(D_METHOD("set_character", "character"), &EPASSoftnessNode::set_character);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "influence"), "set_influence", "get_influence");
}

#ifdef DEBUG_ENABLED
void EPASSoftnessNode::_debug_node_draw() const {
	if (ImPlot::BeginPlot(vformat("acceleration\n%.2f", acceleration_plot[1][PLOT_SAMPLES - 1]).utf8().get_data(), ImVec2(300, 200))) {
		ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels);
		ImPlot::SetupAxisLimits(ImAxis_X1, acceleration_plot[0][0], acceleration_plot[0][PLOT_SAMPLES - 1], ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0f, 1.0f, ImGuiCond_Always);
		ImPlot::PlotLine("Acceleration", acceleration_plot[0], acceleration_plot[1], PLOT_SAMPLES);
		ImPlot::EndPlot();
	}

	ImGui::PushItemWidth(100.0f);
	ImGui::SliderFloat("Influence", const_cast<float *>(&influence), 0.0f, 1.0f);
	ImGui::PopItemWidth();
}
#endif

void EPASSoftnessNode::set_bone_softness(const StringName &p_bone_name, float p_softness) {
	if (p_softness == 0) {
		if (softness_map.has(p_bone_name)) {
			softness_map.erase(p_bone_name);
		}
		return;
	}
	if (softness_map.has(p_bone_name)) {
		softness_map[p_bone_name].softness = p_softness;
	} else {
		SoftnessMapEntry entry;
		entry.softness = p_softness;
		softness_map.insert(p_bone_name, entry);
	}
}

void EPASSoftnessNode::set_character(HBAgent *p_agent) {
	character = p_agent;
}

void EPASSoftnessNode::set_influence(float p_influence) {
	influence = p_influence;
}

float EPASSoftnessNode::get_influence() const {
	return influence;
}

void EPASSoftnessNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	process_input_pose(0, p_base_pose, p_target_pose, p_delta);
	return;
	/*if (influence <= 0.0f) {
		return;
	}*/

	plot_t += p_delta;
	for (int i = 1; i < PLOT_SAMPLES; i++) {
		acceleration_plot[0][i - 1] = acceleration_plot[0][i];
		acceleration_plot[1][i - 1] = acceleration_plot[1][i];
	}

	acceleration_plot[0][PLOT_SAMPLES - 1] = plot_t;

	Vector3 accel = (character->get_velocity() - character_prev_velocities[0]);
	float accel_quantity = (accel / p_delta).length();
	accel_quantity = MIN(accel_quantity, 1.0f);
	//accel_quantity *= acceleration_multiplier;
	//accel_quantity = MIN(accel_quantity, 1.0f);
	//float accel_quantity = MAX(0.0f, accel.dot(forward));
	//accel_quantity = MIN(accel_quantity * acceleration_multiplier, 1.0f);
	acceleration_plot[1][PLOT_SAMPLES - 1] = accel_quantity;
	character_prev_velocities[1] = character_prev_velocities[0];
	character_prev_velocities[0] = character->get_velocity();

	for (KeyValue<StringName, SoftnessMapEntry> &kv : softness_map) {
		StringName bone_name = kv.key;
		float softness = kv.value.softness * influence;
		int bone_idx = get_skeleton()->find_bone(bone_name);
		int parent_bone_idx = get_skeleton()->get_bone_parent(bone_idx);
		StringName parent_bone_name = get_skeleton()->get_bone_name(parent_bone_idx);
		int child_idx = get_skeleton()->get_bone_children(bone_idx).get(0);
		StringName child_bone_name = get_skeleton()->get_bone_name(child_idx);

		// Skeleton has the previous transforms of the bones, so we use those
		const Transform3D prev_child_bone_trf = get_skeleton()->get_global_transform() * get_skeleton()->get_bone_global_pose(child_idx);

		if (!kv.value.has_global_rot) {
			kv.value.has_global_rot = true;
			kv.value.child_spring_position = prev_child_bone_trf.origin;
		}

		// Bone transform if we were to apply the animation without softness
		const Transform3D parent_bone_trf = get_skeleton()->get_global_transform() * p_target_pose->calculate_bone_global_transform(parent_bone_name, get_skeleton(), p_base_pose);
		const Transform3D target_bone_trf = get_skeleton()->get_global_transform() * p_target_pose->calculate_bone_global_transform(bone_name, get_skeleton(), p_base_pose);
		const Transform3D child_bone_trf = get_skeleton()->get_global_transform() * p_target_pose->calculate_bone_global_transform(child_bone_name, get_skeleton(), p_base_pose);

		Vector3 new_child_spring_position = kv.value.child_spring_position;
		/*HBSprings::tracking_spring_update_exact_vector3(
			new_child_spring_position,
			kv.value.child_spring_velocity,
			child_bone_trf.origin,
			target_vel,
			target_accel,
			position_gain,
			velocity_gain,
			acceleration_gain,
			p_delta,
			1.0f / 60.0f);
		*/
		HBSprings::critical_spring_damper_exact_vector3(
				new_child_spring_position,
				kv.value.child_spring_velocity,
				child_bone_trf.origin,
				softness * accel_quantity, p_delta);
		kv.value.child_spring_position = new_child_spring_position;

		Vector3 direction_to_old_child = target_bone_trf.origin.direction_to(new_child_spring_position);
		Vector3 direction_to_new_child = target_bone_trf.origin.direction_to(child_bone_trf.origin);

		Quaternion offset = Quaternion(direction_to_new_child, direction_to_old_child);
		Quaternion target_rotation = offset * target_bone_trf.basis;

		if (!p_target_pose->has_bone(bone_name)) {
			p_target_pose->create_bone(bone_name);
		}
		p_target_pose->set_bone_rotation(bone_name, parent_bone_trf.basis.inverse() * target_rotation);
		// Shuffle previous positions
		kv.value.child_prev_positions[1] = kv.value.child_prev_positions[0];
		kv.value.child_prev_positions[0] = child_bone_trf.origin;
	}
}

EPASSoftnessNode::EPASSoftnessNode() {
	_set_input_count(1);
}
