#include "epas_oneshot_animation_node.h"

void EPASOneshotAnimationNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_root_bone", "root_bone"), &EPASOneshotAnimationNode::set_root_bone);
	ClassDB::bind_method(D_METHOD("get_root_bone"), &EPASOneshotAnimationNode::get_root_bone);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "root_bone"), "set_root_bone", "get_root_bone");

	ClassDB::bind_method(D_METHOD("set_animation", "animation"), &EPASOneshotAnimationNode::set_animation);
	ClassDB::bind_method(D_METHOD("get_animation"), &EPASOneshotAnimationNode::get_animation);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "animation", PROPERTY_HINT_RESOURCE_TYPE, "EPASAnimation"), "set_animation", "get_animation");

	ClassDB::bind_method(D_METHOD("play"), &EPASOneshotAnimationNode::play);
	ClassDB::bind_method(D_METHOD("is_playing"), &EPASOneshotAnimationNode::is_playing);

	ClassDB::bind_method(D_METHOD("get_use_root_motion"), &EPASOneshotAnimationNode::get_use_root_motion);
	ClassDB::bind_method(D_METHOD("set_use_root_motion", "use_root_motion"), &EPASOneshotAnimationNode::set_use_root_motion);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_root_motion"), "set_use_root_motion", "get_use_root_motion");

	ClassDB::bind_method(D_METHOD("get_root_motion_starting_transform"), &EPASOneshotAnimationNode::get_root_motion_starting_transform);
	ClassDB::bind_method(D_METHOD("set_root_motion_starting_transform", "root_motion_starting_transform"), &EPASOneshotAnimationNode::set_root_motion_starting_transform);
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "root_motion_starting_transform"), "set_root_motion_starting_transform", "get_root_motion_starting_transform");

	ClassDB::bind_method(D_METHOD("get_root_motion_forward"), &EPASOneshotAnimationNode::get_root_motion_forward);
	ClassDB::bind_method(D_METHOD("set_root_motion_forward", "root_motion_forward"), &EPASOneshotAnimationNode::set_root_motion_forward);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "root_motion_forward"), "set_root_motion_forward", "get_root_motion_forward");

	ClassDB::bind_method(D_METHOD("get_root_motion_transform"), &EPASOneshotAnimationNode::get_root_motion_transform);

	ClassDB::bind_method(D_METHOD("set_warp_point_transform", "name", "transform"), &EPASOneshotAnimationNode::set_warp_point_transform);

	ADD_SIGNAL(MethodInfo("playback_finished"));
}

void EPASOneshotAnimationNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	if (animation.is_valid() && playing) {
		time += p_delta;
		float sample_time = MIN(time, animation->get_length());
		animation->interpolate(sample_time, p_base_pose, p_target_pose, interpolation_method, &playback_info);
		if (time >= animation->get_length()) {
			emit_signal(SNAME("playback_finished"));
			playing = false;
		}
	}
}

void EPASOneshotAnimationNode::play() {
	time = 0.0f;
	playing = true;
	playback_info.root_motion_trf = Transform3D();
}

bool EPASOneshotAnimationNode::is_playing() const {
	return playing;
}

Ref<EPASAnimation> EPASOneshotAnimationNode::get_animation() const {
	return animation;
}

void EPASOneshotAnimationNode::set_animation(const Ref<EPASAnimation> &p_animation) {
	animation = p_animation;
}

bool EPASOneshotAnimationNode::get_use_root_motion() const {
	return playback_info.use_root_motion;
}

void EPASOneshotAnimationNode::set_use_root_motion(bool p_use_root_motion) {
	playback_info.use_root_motion = p_use_root_motion;
}

StringName EPASOneshotAnimationNode::get_root_bone() const {
	return playback_info.root_bone;
}

void EPASOneshotAnimationNode::set_root_bone(const StringName &p_root_bone) {
	playback_info.root_bone = p_root_bone;
}

void EPASOneshotAnimationNode::set_root_motion_starting_transform(const Transform3D &p_root_motion_starting_transform) {
	playback_info.starting_global_trf = p_root_motion_starting_transform;
}

Transform3D EPASOneshotAnimationNode::get_root_motion_starting_transform() const {
	return playback_info.starting_global_trf;
}

void EPASOneshotAnimationNode::set_root_motion_forward(const Vector3 &p_root_motion_forward) {
	playback_info.forward = p_root_motion_forward;
}

Vector3 EPASOneshotAnimationNode::get_root_motion_forward() const {
	return playback_info.forward;
}

void EPASOneshotAnimationNode::set_warp_point_transform(const StringName &p_name, const Transform3D &p_transform) {
	playback_info.warp_point_transforms[p_name] = p_transform;
}

Transform3D EPASOneshotAnimationNode::get_root_motion_transform() const {
	return playback_info.root_motion_trf;
}
