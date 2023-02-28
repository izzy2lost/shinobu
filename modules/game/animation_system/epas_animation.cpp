#include "epas_animation.h"
#include "../utils.h"
#include "core/error/error_macros.h"
#include "core/object/callable_method_pointer.h"
#include "core/object/class_db.h"
#include "core/object/object.h"

#pragma region Keyframe

void EPASKeyframe::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_pose", "pose"), &EPASKeyframe::set_pose);
	ClassDB::bind_method(D_METHOD("get_pose"), &EPASKeyframe::get_pose);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "pose", PROPERTY_HINT_RESOURCE_TYPE, "EPASPose"), "set_pose", "get_pose");

	ClassDB::bind_method(D_METHOD("set_time", "time"), &EPASKeyframe::set_time);
	ClassDB::bind_method(D_METHOD("get_time"), &EPASKeyframe::get_time);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time"), "set_time", "get_time");

	ADD_SIGNAL(MethodInfo("time_changed"));
}

Ref<EPASPose> EPASKeyframe::get_pose() const {
	return pose;
}

void EPASKeyframe::set_pose(const Ref<EPASPose> &p_pose) {
	pose = p_pose;
}

float EPASKeyframe::get_time() const {
	return time;
}

void EPASKeyframe::set_time(float p_time) {
	time = p_time;
	emit_signal("time_changed");
}

#pragma endregion Keyframe

#pragma region Animation

Array EPASAnimation::_get_keyframes() const {
	Array out;
	out.resize(keyframes.size());
	for (int i = 0; i < keyframes.size(); i++) {
		out.set(i, keyframes[i]);
	}
	return out;
}

void EPASAnimation::_keyframe_time_changed() {
	keyframe_order_dirty = true;
	length_cache_dirty = true;
}

void EPASAnimation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_keyframe", "keyframe"), &EPASAnimation::add_keyframe);
	ClassDB::bind_method(D_METHOD("_set_keyframes", "keyframes"), &EPASAnimation::_set_keyframes);
	ClassDB::bind_method(D_METHOD("_get_keyframes"), &EPASAnimation::_get_keyframes);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "keyframes", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL), "_set_keyframes", "_get_keyframes");

	BIND_ENUM_CONSTANT(STEP);
	BIND_ENUM_CONSTANT(LINEAR);
	BIND_ENUM_CONSTANT(BICUBIC_SPLINE);
}

void EPASAnimation::_sort_keyframes() {
	keyframes.sort_custom<EPASKeyframeComparator>();
	keyframe_order_dirty = false;
}

void EPASAnimation::_update_length_cache() {
	length_cache = 0.0f;
	if (keyframes.size() > 0) {
		if (keyframe_order_dirty) {
			_sort_keyframes();
		}
		length_cache = keyframes[keyframes.size() - 1]->get_time();
	}
	length_cache_dirty = false;
}

void EPASAnimation::_set_keyframes(const Array &p_keyframes) {
	clear();
	// _set_keyframes is only called when the resource is (re)loaded and replaces all frames
	// since the length cache is also updated by add_keyframe it is guaranteed to not be dirty
	length_cache = 0.0f;
	for (int i = 0; i < p_keyframes.size(); i++) {
		Ref<EPASKeyframe> kf = p_keyframes[i];
		if (kf.is_valid()) {
			add_keyframe(kf);
		}
	}
	length_cache_dirty = false;
}

void EPASAnimation::add_keyframe(Ref<EPASKeyframe> p_keyframe) {
	ERR_FAIL_COND_MSG(keyframes.has(p_keyframe), "Keyframe is already in animation");
	keyframes.push_back(p_keyframe);
	p_keyframe->connect("time_changed", callable_mp(this, &EPASAnimation::_keyframe_time_changed));
	length_cache = MAX(p_keyframe->get_time(), length_cache);
	keyframe_order_dirty = true;
}

void EPASAnimation::erase_keyframe(Ref<EPASKeyframe> p_keyframe) {
	ERR_FAIL_COND_MSG(!keyframes.has(p_keyframe), "Keyframe was not in this animation");
	p_keyframe->disconnect("time_changed", callable_mp(this, &EPASAnimation::_keyframe_time_changed));
	keyframes.erase(p_keyframe);
	length_cache_dirty = true;
}

int EPASAnimation::get_keyframe_count() const {
	return keyframes.size();
}

Ref<EPASKeyframe> EPASAnimation::get_keyframe(int p_idx) const {
	ERR_FAIL_INDEX_V_MSG(p_idx, keyframes.size(), Ref<EPASKeyframe>(), "Keyframe index is out of range");
	return keyframes[p_idx];
}

void EPASAnimation::interpolate(float p_time, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, InterpolationMethod p_interp_method) const {
	if (keyframes.size() == 0) {
		// do nothing
		return;
	} else if (keyframes.size() == 1) {
		// A bit hacky but eehhhh
		keyframes[0]->get_pose()->blend(keyframes[0]->get_pose(), p_base_pose, p_target_pose, 0.0f);
		return;
	}
	int prev_frame_i = -1;
	int next_frame_i = -1;

	if (keyframe_order_dirty) {
		const_cast<EPASAnimation *>(this)->_sort_keyframes();
	}

	for (int i = 0; i < keyframes.size() - 1; i++) {
		if (keyframes[i]->get_time() <= p_time) {
			next_frame_i = (i + 1) % keyframes.size();
			prev_frame_i = i;
		}
	}

	ERR_FAIL_COND_MSG(prev_frame_i == -1, "Animation or interpolation time are invalid.");

	float blend_start = keyframes[prev_frame_i]->get_time();
	float blend_end = keyframes[next_frame_i]->get_time();
	float blend = Math::inverse_lerp(blend_start, blend_end, MIN(p_time, blend_end));

	switch (p_interp_method) {
		case STEP: {
			// TODO: implement this
			keyframes[prev_frame_i]->get_pose()->blend(keyframes[next_frame_i]->get_pose(), p_base_pose, p_target_pose, blend_start);
		} break;
		case LINEAR: {
			keyframes[prev_frame_i]->get_pose()->blend(keyframes[next_frame_i]->get_pose(), p_base_pose, p_target_pose, blend);
		} break;
		case BICUBIC_SPLINE: {
			// Do a cubic spline interpolation thingy
			// gotta be honest with you i have no idea how this works
			float weights[4];
			HBUtils::get_cubic_spline_weights(blend, weights);
			float total_weight = weights[0] + weights[1];
			int frames[4];
			frames[0] = Math::posmod(next_frame_i - 1, keyframes.size());
			frames[1] = prev_frame_i;
			frames[2] = next_frame_i;
			frames[3] = Math::posmod(next_frame_i + 1, keyframes.size());
			if (total_weight > 0.0f) {
				keyframes[frames[0]]->get_pose()->blend(keyframes[frames[1]]->get_pose(), p_base_pose, p_target_pose, weights[1] / total_weight);
			}
			total_weight += weights[2];
			if (total_weight > 0.0f) {
				p_target_pose->blend(keyframes[frames[2]]->get_pose(), p_base_pose, p_target_pose, weights[2] / total_weight);
			}
			total_weight += weights[3];
			if (total_weight > 0.0f) {
				p_target_pose->blend(keyframes[frames[3]]->get_pose(), p_base_pose, p_target_pose, weights[3] / total_weight);
			}
		} break;
	};
}

float EPASAnimation::get_length() const {
	if (length_cache_dirty) {
		const_cast<EPASAnimation *>(this)->_update_length_cache();
	}
	return length_cache;
}

void EPASAnimation::clear() {
	for (int i = 0; i < keyframes.size(); i++) {
		keyframes.get(i)->disconnect("time_changed", callable_mp(this, &EPASAnimation::_keyframe_time_changed));
	}
	keyframes.clear();
}

#pragma endregion Animation
