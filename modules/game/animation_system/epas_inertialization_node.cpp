#include "epas_inertialization_node.h"
#include "epas_animation.h"
#include "modules/game/animation_system/epas_animation_editor.h"

void EPASInertializationNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("inertialize", "transition_duration", "bone_filter"), &EPASInertializationNode::inertialize, DEFVAL(0.25f), DEFVAL(TypedArray<StringName>()));
}

void remove_pose_root_motion(Ref<EPASPose> p_pose, Ref<EPASPose> p_base_pose) {
	ERR_FAIL_COND(!p_pose.is_valid());
	StringName root_bone = SNAME("root");
	if (!p_pose->has_bone(root_bone)) {
		return;
	}
	p_pose->set_bone_position(root_bone, p_base_pose->get_bone_position(root_bone));
	p_pose->set_bone_rotation(root_bone, p_base_pose->get_bone_rotation(root_bone));
}

/*void quick_dump_pose_to_anim(Ref<EPASPose> p_pose, const String &p_path) {
	Ref<EPASAnimation> polla;
	polla.instantiate();
	Ref<EPASKeyframe> kf1 = memnew(EPASKeyframe);
	kf1->set_time(0.0f);
	kf1->set_pose(p_pose);

	polla->add_keyframe(kf1);
	ResourceSaver::save(polla, p_path);
}*/

void EPASInertializationNode::start_inertialization(const Ref<EPASPose> &p_base_pose, const Ref<EPASPose> &p_target_pose, float p_delta) {
	Vector<Ref<EPASPose>> poses;
	remove_pose_root_motion(last_last_frame_pose, p_base_pose);
	remove_pose_root_motion(last_frame_pose, p_base_pose);
	remove_pose_root_motion(p_target_pose, p_base_pose);
	poses.resize(EPASPoseInertializer::InertializationPose::POSE_MAX);
	poses.set(EPASPoseInertializer::InertializationPose::PREV_PREV_POSE, last_last_frame_pose);
	poses.set(EPASPoseInertializer::InertializationPose::PREV_POSE, last_frame_pose);
	poses.set(EPASPoseInertializer::InertializationPose::TARGET_POSE, p_target_pose);

	/*Ref<EPASEditorAnimation> polla;
	polla.instantiate();
	Ref<EPASKeyframe> kf1 = memnew(EPASKeyframe);
	kf1->set_time(0.0f);
	kf1->set_pose(p_target_pose);
	Ref<EPASKeyframe> kf2 = memnew(EPASKeyframe);
	kf2->set_time(0.5f);
	kf2->set_pose(last_frame_pose);
	Ref<EPASKeyframe> kf3 = memnew(EPASKeyframe);
	kf3->set_time(1.0f);
	kf3->set_pose(last_last_frame_pose);

	polla->add_keyframe(kf1);
	polla->add_keyframe(kf2);
	polla->add_keyframe(kf3);
	ResourceSaver::save(polla, "res://inertialization_dumped.tres");
	*/
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
		// Inertialization is all about going to a target pose, so we want to use the pose the user just set as
		// our target, otherwise the transition will have a jump (if we apply an already existing inertialization step)
		Ref<EPASPose> inert_target_pose;
		inert_target_pose.instantiate();
		process_input_pose(0, p_base_pose, inert_target_pose, p_delta);
		start_inertialization(p_base_pose, inert_target_pose, p_delta);
		inertialization_queued = false;
		print_line(vformat("Fulfilled inertialization on frame %d %.2f", Engine::get_singleton()->get_frames_drawn(), pose_inertializer->get_current_transition_time()));
	}
	process_input_pose_inertialized(0, p_base_pose, p_target_pose, delta);
	last_last_frame_pose = last_frame_pose;
	last_frame_pose = p_target_pose->duplicate();
}

void EPASInertializationNode::inertialize(float p_transition_duration, TypedArray<StringName> p_bone_filter) {
	print_line(vformat("Requested inertialization on frame %d for time %.2f", Engine::get_singleton()->get_frames_drawn(), p_transition_duration), inertialization_queued);
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