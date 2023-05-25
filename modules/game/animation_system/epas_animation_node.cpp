#include "epas_animation_node.h"
#include "core/object/class_db.h"
#include "core/object/object.h"
#include "imgui.h"
#include "modules/game/animation_system/epas_node.h"
#include "scene/resources/animation.h"

void EPASAnimationNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_animation", "animation"), &EPASAnimationNode::set_animation);
	ClassDB::bind_method(D_METHOD("get_animation"), &EPASAnimationNode::get_animation);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "animation", PROPERTY_HINT_RESOURCE_TYPE, "EPASAnimation"), "set_animation", "get_animation");

	ClassDB::bind_method(D_METHOD("set_playback_mode", "playback_mode"), &EPASAnimationNode::set_playback_mode);
	ClassDB::bind_method(D_METHOD("get_playback_mode"), &EPASAnimationNode::get_playback_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "playback_mode", PROPERTY_HINT_ENUM, "Automatic,Manual"), "set_playback_mode", "get_playback_mode");

	ClassDB::bind_method(D_METHOD("set_interpolation_method", "interpolation_method"), &EPASAnimationNode::set_interpolation_method);
	ClassDB::bind_method(D_METHOD("get_interpolation_method"), &EPASAnimationNode::get_interpolation_method);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "interpolation_method", PROPERTY_HINT_ENUM, "Step,Linear,Bicubic,Bicubic Clamped"), "set_interpolation_method", "get_interpolation_method");

	ClassDB::bind_method(D_METHOD("set_looping_enabled", "looping_enabled"), &EPASAnimationNode::set_looping_enabled);
	ClassDB::bind_method(D_METHOD("get_looping_enabled"), &EPASAnimationNode::get_looping_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "looping_enabled"), "set_looping_enabled", "get_looping_enabled");

	BIND_ENUM_CONSTANT(EPASAnimationNode::AUTOMATIC);
	BIND_ENUM_CONSTANT(EPASAnimationNode::MANUAL);

	ADD_SIGNAL(MethodInfo("playback_finished"));
}

#ifdef DEBUG_ENABLED

#include "modules/imgui/godot_imgui.h"

void EPASAnimationNode::_debug_node_draw() const {
	if (!animation.is_valid()) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0, 1.0, 0.0, 1.0));
		ImGui::TextUnformatted("!!WARNING!!");
		ImGui::TextUnformatted("No animation");
		ImGui::PopStyleColor();
	}
	ImGui::Text("Time: %.2f s", time);
	String interp_type;
	switch (interpolation_method) {
		case EPASAnimation::InterpolationMethod::STEP: {
			interp_type = "Step";
		} break;
		case EPASAnimation::InterpolationMethod::LINEAR: {
			interp_type = "Linear";
		} break;
		case EPASAnimation::InterpolationMethod::BICUBIC_SPLINE: {
			interp_type = "Bicubic";
		} break;
		case EPASAnimation::InterpolationMethod::BICUBIC_SPLINE_CLAMPED: {
			interp_type = "Bicubic Clamped";
		} break;
	}
	ImGui::Text("Interp: %s", interp_type.utf8().get_data());
}
#endif

void EPASAnimationNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	if (animation.is_valid()) {
		if (playback_mode == PlaybackMode::AUTOMATIC) {
			time += p_delta;
		}
		if (looping_enabled) {
			time = Math::fposmod(time, animation->get_length());
		} else {
			time = MIN(time, animation->get_length());
		}
		animation->interpolate(time, p_base_pose, p_target_pose, interpolation_method);
	}
}

void EPASAnimationNode::set_animation(Ref<EPASAnimation> p_animation) {
	animation = p_animation;
}

Ref<EPASAnimation> EPASAnimationNode::get_animation() const {
	return animation;
}

void EPASAnimationNode::set_playback_mode(PlaybackMode p_playback_mode) {
	playback_mode = p_playback_mode;
}

EPASAnimationNode::PlaybackMode EPASAnimationNode::get_playback_mode() const {
	return playback_mode;
}

void EPASAnimationNode::set_interpolation_method(EPASAnimation::InterpolationMethod p_interpolation_method) {
	interpolation_method = p_interpolation_method;
}

EPASAnimation::InterpolationMethod EPASAnimationNode::get_interpolation_method() const {
	return interpolation_method;
}

void EPASAnimationNode::seek(float p_time) {
	time = p_time;
}

float EPASAnimationNode::get_time() {
	return time;
}

bool EPASAnimationNode::get_looping_enabled() const {
	return looping_enabled;
}

void EPASAnimationNode::set_looping_enabled(bool p_looping_enabled) {
	looping_enabled = p_looping_enabled;
}
