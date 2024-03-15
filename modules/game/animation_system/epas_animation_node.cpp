/**************************************************************************/
/*  epas_animation_node.cpp                                               */
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

#include "epas_animation_node.h"
#include "core/object/class_db.h"
#include "core/object/object.h"
#include "scene/resources/audio_stream_polyphonic.h"
#ifdef DEBUG_ENABLED
#include "imgui.h"
#endif
#include "epas_controller.h"
#include "modules/game/animation_system/epas_node.h"
#include "scene/resources/animation.h"

void EPASAnimationNode::_on_animation_event_fired(const Ref<EPASAnimationEvent> &p_event) {
	print_line("PLAY SOUND");
	if (Ref<EPASSoundAnimationEvent> sound = p_event; sound.is_valid()) {
		Ref<AudioStreamPlaybackPolyphonic> playback = get_epas_controller()->get_audio_stream_playback();
		playback->play_stream(sound->get_stream());
	}
}

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

	ClassDB::bind_method(D_METHOD("seek", "time"), &EPASAnimationNode::seek);
	ClassDB::bind_method(D_METHOD("get_time"), &EPASAnimationNode::get_time);

	BIND_ENUM_CONSTANT(AUTOMATIC);
	BIND_ENUM_CONSTANT(MANUAL);

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
		interpolate(p_base_pose, p_target_pose, time);
	}
}

void EPASAnimationNode::interpolate(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_time) {
	animation->interpolate(p_time, p_base_pose, p_target_pose, get_interpolation_method());
}

void EPASAnimationNode::set_animation(Ref<EPASAnimation> p_animation) {
	if (animation.is_valid()) {
		animation->disconnect(SNAME("event_fired"), callable_mp(this, &EPASAnimationNode::_on_animation_event_fired));
	}
	animation = p_animation;
	animation->connect(SNAME("event_fired"), callable_mp(this, &EPASAnimationNode::_on_animation_event_fired));
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
