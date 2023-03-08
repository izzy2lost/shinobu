#include "epas_inertialization_node.h"
#include "../utils.h"

void EPASInertializationNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("inertialize", "transition_duration", "bone_filter"), &EPASInertializationNode::inertialize, DEFVAL(0.25f), DEFVAL(TypedArray<StringName>()));
}

void EPASInertializationNode::start_inertialization(const Ref<EPASPose> &p_base_pose, const Ref<EPASPose> &p_target_pose, float p_delta) {
	transition_infos.clear();
	transition_time = 0.0f;
	for (int i = 0; i < p_base_pose->get_bone_count(); i++) {
		StringName bone_name = p_base_pose->get_bone_name(i);
		if (bone_filter.size() > 0 && !bone_filter.has(bone_name)) {
			continue;
		}
		TransitionInfo ti;
		ti.bone_name = bone_name;
		{
			// Position info
			Vector3 x_prev = last_frame_pose->get_bone_position(bone_name, p_base_pose) - p_target_pose->get_bone_position(bone_name, p_base_pose);
			Vector3 x_prev_prev = last_last_frame_pose->get_bone_position(bone_name, p_base_pose) - p_target_pose->get_bone_position(bone_name, p_base_pose);
			float x_m_1 = x_prev_prev.dot(x_prev.normalized());
			ti.pos_velocity = MIN((x_prev.length() - x_m_1) / p_delta, 0.0f);
			ti.pos_offset = x_prev;
			ti.pos_blend_time = desired_blend_time;
			if (ti.pos_velocity != 0.0f) {
				ti.pos_blend_time = MIN(desired_blend_time, -5.0f * (x_prev.length() / ti.pos_velocity));
			}
		}
		{
			// Rotation info
			Quaternion q_prev = last_frame_pose->get_bone_rotation(bone_name, p_base_pose) * p_target_pose->get_bone_rotation(bone_name, p_base_pose).inverse();
			Quaternion q_prev_prev = last_last_frame_pose->get_bone_rotation(bone_name, p_base_pose) * p_target_pose->get_bone_rotation(bone_name, p_base_pose).inverse();
			Vector3 x0_axis;
			float x0_angle;
			q_prev.get_axis_angle(x0_axis, x0_angle);
			x0_axis.normalize();
			if (x0_axis.is_normalized()) {
				Vector3 q_x_y_z = Vector3(q_prev_prev.x, q_prev_prev.y, q_prev_prev.z);
				float q_x_m_1 = 2.0f * Math::atan(q_x_y_z.dot(x0_axis) / q_prev_prev.w);
				ti.rot_velocity = MIN((x0_angle - q_x_m_1) / p_delta, 0.0f);
				ti.rot_offset = q_prev;
				ti.rot_blend_time = desired_blend_time;
				if (ti.rot_velocity != 0.0f) {
					ti.rot_blend_time = MIN(desired_blend_time, -5.0f * x0_angle / ti.rot_velocity);
				}
			}
		}
		if (Math::is_zero_approx(ti.rot_offset.get_angle()) && Math::is_zero_approx(ti.pos_offset.length())) {
			continue;
		}
		transition_infos.push_back(ti);
	}
}

bool EPASInertializationNode::process_inertialization(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> &p_target_pose) {
	bool done = true;
	for (int i = 0; i < transition_infos.size(); i++) {
		const TransitionInfo &ti = transition_infos[i];

		bool is_pos_done = transition_time >= ti.pos_blend_time;
		bool is_rot_done = transition_time >= ti.rot_blend_time;

		if (!is_pos_done || !is_rot_done) {
			done = false;
		}

		if (is_pos_done && is_rot_done) {
			continue;
		}

		if (!p_target_pose->has_bone(ti.bone_name)) {
			p_target_pose->create_bone(ti.bone_name);
		}
		if (!is_pos_done) {
			float pos_x = HBUtils::inertialize(ti.pos_offset.length(), ti.pos_velocity, ti.pos_blend_time, transition_time);
			Vector3 pos_off = ti.pos_offset.normalized() * pos_x;
			Vector3 bone_pos = p_target_pose->get_bone_position(ti.bone_name, p_base_pose) + pos_off;
			p_target_pose->set_bone_position(ti.bone_name, bone_pos);
		}

		if (!is_rot_done) {
			float rot_x = HBUtils::inertialize(ti.rot_offset.get_angle(), ti.rot_velocity, ti.rot_blend_time, transition_time);
			Quaternion rot_off = Quaternion(ti.rot_offset.get_axis().normalized(), rot_x);
			Quaternion bone_rot = rot_off * p_target_pose->get_bone_rotation(ti.bone_name, p_base_pose);
			p_target_pose->set_bone_rotation(ti.bone_name, bone_rot);
		}
	}
	return done;
}

void EPASInertializationNode::process_input_pose_inertialized(int p_input, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> &p_target_pose, float p_delta) {
	process_input_pose(p_input, p_base_pose, p_target_pose, p_delta);
	if (transition_infos.size() > 0) {
		if (process_inertialization(p_base_pose, p_target_pose)) {
			transition_infos.clear();
		}
	}
}
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
void EPASInertializationNode::_debug_node_draw() const {
	if (transition_infos.size() > 0) {
		ImGui::Text("%.2f", transition_time);
	} else {
		ImGui::TextUnformatted("Idle");
	}
}
#endif

void EPASInertializationNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	if (transition_infos.size() > 0) {
		transition_time += p_delta;
	}

	process_input_pose_inertialized(0, p_base_pose, p_target_pose, p_delta);
	// Start calculating inertialization information
	if (inertialization_queued && last_frame_pose.is_valid() && last_last_frame_pose.is_valid()) {
		process_input_pose_inertialized(0, p_base_pose, p_target_pose, p_delta);
		start_inertialization(p_base_pose, p_target_pose, p_delta);
		inertialization_queued = false;
	}
	process_input_pose_inertialized(0, p_base_pose, p_target_pose, p_delta);
	last_last_frame_pose = last_frame_pose;
	last_frame_pose = p_target_pose->duplicate();
}

void EPASInertializationNode::inertialize(float p_transition_duration, TypedArray<StringName> p_bone_filter) {
	inertialization_queued = true;
	desired_blend_time = p_transition_duration;
	bone_filter = p_bone_filter;
}

EPASInertializationNode::EPASInertializationNode() {
	_set_input_count(1);
}