#include "epas_inertialization_node.h"

void EPASInertializationNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("inertialize", "transition_duration", "bone_filter"), &EPASInertializationNode::inertialize, DEFVAL(0.25f), DEFVAL(TypedArray<StringName>()));
}

void EPASInertializationNode::start_inertialization(const Ref<EPASPose> &p_base_pose, const Ref<EPASPose> &p_target_pose, float p_delta) {
	Vector<Ref<EPASPose>> poses;
	poses.resize(EPASPoseInertializer::InertializationPose::POSE_MAX);
	poses.set(EPASPoseInertializer::InertializationPose::PREV_PREV_POSE, last_last_frame_pose);
	poses.set(EPASPoseInertializer::InertializationPose::PREV_POSE, last_frame_pose);
	poses.set(EPASPoseInertializer::InertializationPose::TARGET_POSE, p_target_pose);

	pose_inertializer = EPASPoseInertializer::create(poses.ptr(), p_base_pose, desired_blend_time, p_delta, bone_filter);
}

void EPASInertializationNode::process_input_pose_inertialized(int p_input, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> &p_target_pose, float p_delta) {
	process_input_pose(p_input, p_base_pose, p_target_pose, p_delta);
	if (pose_inertializer.is_valid()) {
		if (pose_inertializer->advance(p_target_pose, p_base_pose, p_delta)) {
			pose_inertializer = Ref<EPASPoseInertializer>();
		}
	}
}
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
void EPASInertializationNode::_debug_node_draw() const {
	if (pose_inertializer.is_valid()) {
		ImGui::Text("%f", pose_inertializer->get_current_transition_time());
	} else {
		ImGui::TextUnformatted("Idle");
	}
}
#endif

void EPASInertializationNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	float delta = p_delta;
	// Start calculating inertialization information
	if (inertialization_queued && last_frame_pose.is_valid() && last_last_frame_pose.is_valid()) {
		process_input_pose_inertialized(0, p_base_pose, p_target_pose, p_delta);
		start_inertialization(p_base_pose, p_target_pose, p_delta);
		inertialization_queued = false;
		delta = 0.0f;
	}
	process_input_pose_inertialized(0, p_base_pose, p_target_pose, delta);
	last_last_frame_pose = last_frame_pose;
	last_frame_pose = p_target_pose->duplicate();
}

void EPASInertializationNode::inertialize(float p_transition_duration, TypedArray<StringName> p_bone_filter) {
	inertialization_queued = true;
	desired_blend_time = p_transition_duration;
	bone_filter = p_bone_filter;
}

bool EPASInertializationNode::is_inertializing() const {
	return pose_inertializer.is_valid() || inertialization_queued;
}

EPASInertializationNode::EPASInertializationNode() {
	_set_input_count(1);
}