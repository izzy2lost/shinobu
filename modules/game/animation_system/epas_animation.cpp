#include "epas_animation.h"
#include "core/error/error_macros.h"
#include "core/object/callable_method_pointer.h"
#include "core/object/class_db.h"
#include "core/object/object.h"

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

Array EPASAnimation::_get_keyframes() const {
	Array out;
	out.resize(keyframes.size());
	for (int i = 0; i < keyframes.size(); i++) {
		out.set(i, keyframes[i]);
	}
	return out;
}

void EPASAnimation::_set_warp_points(const Array &p_warp_points) {
	warp_points.clear();
	for (int i = 0; i < p_warp_points.size(); i++) {
		Ref<EPASWarpPoint> wp = p_warp_points[i];
		if (wp.is_valid()) {
			add_warp_point(wp);
		}
	}
}

Array EPASAnimation::_get_warp_points() const {
	Array out;
	for (int i = 0; i < warp_points.size(); i++) {
		out.push_back(warp_points[i]);
	}
	return out;
}

void EPASAnimation::_set_animation_curves(const Dictionary &p_animation_curves) {
	for (int i = 0; i < p_animation_curves.size(); i++) {
		StringName key = p_animation_curves.keys()[i];
		Ref<Curve> curve = p_animation_curves.get(key, Variant());
		if (curve.is_valid()) {
			animation_curves.insert(key, curve);
		}
	}
}

Dictionary EPASAnimation::_get_animation_curves() const {
	Dictionary dict;

	for (KeyValue<StringName, Ref<Curve>> kv : animation_curves) {
		dict[kv.key] = kv.value;
	}

	return dict;
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

	ClassDB::bind_method(D_METHOD("_set_warp_points", "warp_points"), &EPASAnimation::_set_warp_points);
	ClassDB::bind_method(D_METHOD("_get_warp_points"), &EPASAnimation::_get_warp_points);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "warp_points", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL), "_set_warp_points", "_get_warp_points");

	ClassDB::bind_method(D_METHOD("_set_animation_curves", "animation_curves"), &EPASAnimation::_set_animation_curves);
	ClassDB::bind_method(D_METHOD("_get_animation_curves"), &EPASAnimation::_get_animation_curves);
	ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "animation_curves", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL), "_set_animation_curves", "_get_animation_curves");

	ClassDB::bind_method(D_METHOD("get_length"), &EPASAnimation::get_length);

	BIND_ENUM_CONSTANT(STEP);
	BIND_ENUM_CONSTANT(LINEAR);
	BIND_ENUM_CONSTANT(BICUBIC_SPLINE);
	BIND_ENUM_CONSTANT(BICUBIC_SPLINE_CLAMPED);
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
	clear_keyframes();
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

struct EPASWPComparator {
	_FORCE_INLINE_ bool operator()(const Ref<EPASWarpPoint> &a, const Ref<EPASWarpPoint> &b) const {
		int min_a = a->get_translation_start() < 0 ? INT_MAX : a->get_translation_start();
		min_a = MIN(min_a, a->get_rotation_start() < 0 ? INT_MAX : a->get_rotation_start());
		min_a = MIN(min_a, a->get_facing_start() < 0 ? INT_MAX : a->get_facing_start());
		int min_b = b->get_translation_start() < 0 ? INT_MAX : b->get_translation_start();
		min_b = MIN(min_b, b->get_rotation_start() < 0 ? INT_MAX : b->get_rotation_start());
		min_b = MIN(min_b, b->get_facing_start() < 0 ? INT_MAX : b->get_facing_start());
		return (min_a < min_b);
	}
};

static void get_cubic_spline_weights(float interp, float *weights) {
	// Lifted straight from overgrwoth, no idea what this is
	float interp_squared = interp * interp;
	float interp_cubed = interp_squared * interp;
	weights[0] = 0.5f * (-interp_cubed + 2.0f * interp_squared - interp);
	weights[1] = 0.5f * (3.0f * interp_cubed - 5.0f * interp_squared + 2.0f);
	weights[2] = 0.5f * (-3.0f * interp_cubed + 4.0f * interp_squared + interp);
	weights[3] = 0.5f * (interp_cubed - interp_squared);
}

void EPASAnimation::interpolate(float p_time, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, InterpolationMethod p_interp_method, EPASAnimationPlaybackInfo *p_playback_info) const {
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
			keyframes[prev_frame_i]->get_pose()->blend(keyframes[next_frame_i]->get_pose(), p_base_pose, p_target_pose, 0.0f);
		} break;
		case LINEAR: {
			keyframes[prev_frame_i]->get_pose()->blend(keyframes[next_frame_i]->get_pose(), p_base_pose, p_target_pose, blend);
		} break;
		case BICUBIC_SPLINE_CLAMPED:
		case BICUBIC_SPLINE: {
			// Do a cubic spline interpolation thingy
			// gotta be honest with you i have no idea how this works
			float weights[4];
			get_cubic_spline_weights(blend, weights);
			float total_weight = weights[0] + weights[1];
			int frames[4];
			if (p_interp_method == BICUBIC_SPLINE) {
				frames[0] = Math::posmod(prev_frame_i - 1, keyframes.size());
				frames[1] = prev_frame_i;
				frames[2] = next_frame_i;
				frames[3] = Math::posmod(next_frame_i + 1, keyframes.size());
			} else if (p_interp_method == BICUBIC_SPLINE_CLAMPED) {
				// By clamping it this way we ensure that nothing has any influence on it beyond the ends
				frames[0] = CLAMP(prev_frame_i - 1, 0, keyframes.size() - 1);
				frames[1] = CLAMP(prev_frame_i, 0, keyframes.size() - 1);
				frames[2] = CLAMP(next_frame_i, 0, keyframes.size() - 1);
				frames[3] = CLAMP(next_frame_i + 1, 0, keyframes.size() - 1);
			}

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

	if (p_playback_info) {
		if (p_playback_info->use_root_motion) {
			StringName root_bone_name = p_playback_info->root_bone;
			if (p_target_pose->has_bone(root_bone_name)) {
				if (p_interp_method != InterpolationMethod::LINEAR) {
					// make sure root is interpolated linearly
					Vector3 pos_prev = keyframes[prev_frame_i]->get_pose()->get_bone_position(root_bone_name, p_base_pose);
					Quaternion rot_prev = keyframes[prev_frame_i]->get_pose()->get_bone_rotation(root_bone_name, p_base_pose);

					Vector3 pos_next = keyframes[next_frame_i]->get_pose()->get_bone_position(root_bone_name, p_base_pose);
					Quaternion rot_next = keyframes[next_frame_i]->get_pose()->get_bone_rotation(root_bone_name, p_base_pose);

					if (!p_target_pose->has_bone(root_bone_name)) {
						p_target_pose->create_bone(root_bone_name);
					}
					p_target_pose->set_bone_position(root_bone_name, pos_prev.lerp(pos_next, blend));
					p_target_pose->set_bone_rotation(root_bone_name, rot_prev.slerp(rot_next, blend));
				}
				int frame = p_time * 60.0; // TODO: make this framerate configurable? Animation editor only supports 60 fps r/n
				//Transform3D base_root_trf = p_base_pose->get_bone_transform(root_bone_name);
				Transform3D initial_root_trf = get_keyframe(0)->get_pose()->get_bone_transform(root_bone_name, p_base_pose);
				Transform3D base_root_trf = initial_root_trf;
				Vector<Ref<EPASWarpPoint>> sorted_wps;
				sorted_wps.resize(get_warp_point_count());
				for (int i = 0; i < get_warp_point_count(); i++) {
					sorted_wps.set(i, get_warp_point(i));
				}
				sorted_wps.sort_custom<EPASWPComparator>();

				Vector3 translation_offset;
				Quaternion rotation_offset;
				Quaternion facing_rotation;
				for (int i = 0; i < get_warp_point_count(); i++) {
					Ref<EPASWarpPoint> wp = sorted_wps[i];
					StringName wp_name = wp->get_point_name();
					if (!p_playback_info->warp_point_transforms.has(wp_name)) {
						continue;
					}
					const Transform3D local_new_wp_trf = initial_root_trf * (p_playback_info->starting_global_trf.affine_inverse() * p_playback_info->warp_point_transforms[wp_name]);
					if (wp->get_facing_start() <= frame && wp->has_facing()) {
						// Process facing rotation
						int start_frame = wp->get_facing_start();
						int end_frame = wp->get_facing_end();

						float start_time = start_frame / 60.0f;
						float end_time = end_frame / 60.0f;

						float offset_blend = Math::inverse_lerp(start_time, end_time, p_time);
						// Perhaps we need to rotate from the actual root position instead of the base one and keep an accumulator that
						// persists between frames? who knows, I'm not being paid enough for this
						Transform3D transformed_root_trf = base_root_trf;
						transformed_root_trf.origin += translation_offset;
						transformed_root_trf.origin.y = 0.0f;
						Transform3D wp_trf_cpy = local_new_wp_trf;
						wp_trf_cpy.origin.y = 0.0f;
						Quaternion new_lookat = transformed_root_trf.looking_at(wp_trf_cpy.origin).basis.get_rotation_quaternion();
						if (p_playback_info->forward != Vector3(0.0f, 0.0f, -1.0f)) {
							ERR_FAIL_COND(!p_playback_info->forward.is_normalized());
							Quaternion forward_offset = Basis().looking_at(p_playback_info->forward).get_rotation_quaternion();
							new_lookat = forward_offset * new_lookat;
						}
						// Not particularly elegant way of not needing to recalculate facing for older warp points by storing it in playback_info
						facing_rotation = facing_rotation.slerp(new_lookat, CLAMP(offset_blend, 0.0f, 1.0f));
					}
					if (wp->get_rotation_start() <= frame && wp->has_rotation()) {
						// Do rotation
						int start_frame = wp->get_rotation_start();
						int end_frame = wp->get_rotation_end();

						float start_time = start_frame / 60.0f;
						float end_time = end_frame / 60.0f;

						float offset_blend = Math::inverse_lerp(start_time, end_time, p_time);
						Quaternion local_new_wp_rot = facing_rotation.inverse() * local_new_wp_trf.basis.get_rotation_quaternion();
						rotation_offset = rotation_offset.slerp(local_new_wp_rot, CLAMP(offset_blend, 0.0f, 1.0f));
					}
					if (wp->get_translation_start() <= frame && wp->has_translation()) {
						// Do translation
						int start_frame = wp->get_translation_start();
						int end_frame = wp->get_translation_end();

						float start_time = start_frame / 60.0f;
						float end_time = end_frame / 60.0f;

						float offset_blend = Math::inverse_lerp(start_time, end_time, p_time);
						// Convert to local warp point transform
						Vector3 wp_origin = (facing_rotation * rotation_offset).xform(wp->get_transform().origin);
						Vector3 wp_off = local_new_wp_trf.origin - wp_origin;
						translation_offset = translation_offset.lerp(wp_off, CLAMP(offset_blend, 0.0f, 1.0f));
					}
				}
				// We also rotate the root position based on the facing + rotation offsets
				p_target_pose->set_bone_position(root_bone_name, (facing_rotation * rotation_offset).xform(p_target_pose->get_bone_position(root_bone_name, p_base_pose)) + translation_offset);
				p_target_pose->set_bone_rotation(root_bone_name, facing_rotation * rotation_offset * p_target_pose->get_bone_rotation(root_bone_name, p_base_pose));
				p_playback_info->root_motion_trf = initial_root_trf.affine_inverse() * p_target_pose->get_bone_transform(root_bone_name, p_base_pose);
				p_target_pose->set_bone_position(root_bone_name, p_base_pose->get_bone_position(root_bone_name));
				p_target_pose->set_bone_rotation(root_bone_name, p_base_pose->get_bone_rotation(root_bone_name));
				p_target_pose->set_bone_scale(root_bone_name, p_base_pose->get_bone_scale(root_bone_name));
			}
		}
	}
}

float EPASAnimation::get_length() const {
	if (length_cache_dirty) {
		const_cast<EPASAnimation *>(this)->_update_length_cache();
	}
	return length_cache;
}

bool EPASAnimation::has_warp_point(const StringName &p_name) const {
	for (int i = 0; i < warp_points.size(); i++) {
		if (warp_points[i]->get_point_name() == p_name) {
			return true;
		}
	}
	return false;
}

void EPASAnimation::add_warp_point(Ref<EPASWarpPoint> p_warp_point) {
	ERR_FAIL_COND(!p_warp_point.is_valid());
	StringName point_name = p_warp_point->get_point_name();
	ERR_FAIL_COND_MSG(has_warp_point(point_name), vformat("Warp point %s is already in animation", point_name));
	warp_points.push_back(p_warp_point);
}

void EPASAnimation::erase_warp_point(Ref<EPASWarpPoint> p_warp_point) {
	ERR_FAIL_COND(!p_warp_point.is_valid());
	StringName point_name = p_warp_point->get_point_name();
	ERR_FAIL_COND_MSG(!has_warp_point(point_name), vformat("Warp point %s is not in animation", point_name));
	warp_points.erase(p_warp_point);
}

int EPASAnimation::get_warp_point_count() const {
	return warp_points.size();
}

Ref<EPASWarpPoint> EPASAnimation::get_warp_point(int p_idx) const {
	ERR_FAIL_INDEX_V_MSG(p_idx, warp_points.size(), Ref<EPASWarpPoint>(), "Warp point index out of range");
	return warp_points[p_idx];
}

int EPASAnimation::find_warp_point(const StringName &p_name) const {
	int wp_idx = -1;
	for (int i = 0; i < get_warp_point_count(); i++) {
		if (get_warp_point(i)->get_point_name() == p_name) {
			wp_idx = i;
			break;
		}
	}
	return wp_idx;
}

HashMap<StringName, Ref<Curve>> EPASAnimation::get_animation_curves() const {
	return animation_curves;
}

void EPASAnimation::insert_animation_curve(StringName p_name, Ref<Curve> p_curve) {
	animation_curves.insert(p_name, p_curve);
}

Ref<Curve> EPASAnimation::get_animation_curve(StringName p_curve_name) const {
	const Ref<Curve> *curve_ptr = animation_curves.getptr(p_curve_name);
	ERR_FAIL_COND_V_MSG(!curve_ptr, nullptr, vformat("Curve %s not found", p_curve_name));
	return *curve_ptr;
}

bool EPASAnimation::has_animation_curve(StringName p_curve_name) const {
	return animation_curves.has(p_curve_name);
}

void EPASAnimation::clear_keyframes() {
	for (int i = 0; i < keyframes.size(); i++) {
		keyframes.get(i)->disconnect("time_changed", callable_mp(this, &EPASAnimation::_keyframe_time_changed));
	}
	keyframes.clear();
}

void EPASWarpPoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_transform"), &EPASWarpPoint::get_transform);
	ClassDB::bind_method(D_METHOD("set_transform", "transform"), &EPASWarpPoint::set_transform);
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "transform"), "set_transform", "get_transform");

	ClassDB::bind_method(D_METHOD("get_point_name"), &EPASWarpPoint::get_point_name);
	ClassDB::bind_method(D_METHOD("set_point_name", "point_name"), &EPASWarpPoint::set_point_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "point_name"), "set_point_name", "get_point_name");

	ClassDB::bind_method(D_METHOD("get_facing_start"), &EPASWarpPoint::get_facing_start);
	ClassDB::bind_method(D_METHOD("set_facing_start", "facing_start"), &EPASWarpPoint::set_facing_start);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "facing_start"), "set_facing_start", "get_facing_start");

	ClassDB::bind_method(D_METHOD("get_facing_end"), &EPASWarpPoint::get_facing_end);
	ClassDB::bind_method(D_METHOD("set_facing_end", "facing_end"), &EPASWarpPoint::set_facing_end);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "facing_end"), "set_facing_end", "get_facing_end");

	ClassDB::bind_method(D_METHOD("get_rotation_start"), &EPASWarpPoint::get_rotation_start);
	ClassDB::bind_method(D_METHOD("set_rotation_start", "rotation_start"), &EPASWarpPoint::set_rotation_start);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "rotation_start"), "set_rotation_start", "get_rotation_start");

	ClassDB::bind_method(D_METHOD("get_rotation_end"), &EPASWarpPoint::get_rotation_end);
	ClassDB::bind_method(D_METHOD("set_rotation_end", "rotation_end"), &EPASWarpPoint::set_rotation_end);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "rotation_end"), "set_rotation_end", "get_rotation_end");

	ClassDB::bind_method(D_METHOD("get_translation_start"), &EPASWarpPoint::get_translation_start);
	ClassDB::bind_method(D_METHOD("set_translation_start", "translation_start"), &EPASWarpPoint::set_translation_start);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "translation_start"), "set_translation_start", "get_translation_start");

	ClassDB::bind_method(D_METHOD("get_translation_end"), &EPASWarpPoint::get_translation_end);
	ClassDB::bind_method(D_METHOD("set_translation_end", "translation_end"), &EPASWarpPoint::set_translation_end);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "translation_end"), "set_translation_end", "get_translation_end");
}

Transform3D EPASWarpPoint::get_transform() const {
	return transform;
}

void EPASWarpPoint::set_transform(const Transform3D &p_transform) {
	transform = p_transform;
}

StringName EPASWarpPoint::get_point_name() const {
	return point_name;
}

void EPASWarpPoint::set_point_name(const StringName &p_point_name) {
	point_name = p_point_name;
}

int EPASWarpPoint::get_facing_start() const {
	return facing_start;
}

void EPASWarpPoint::set_facing_start(int p_facing_start) {
	facing_start = p_facing_start;
}

int EPASWarpPoint::get_facing_end() const {
	return facing_end;
}

void EPASWarpPoint::set_facing_end(int p_facing_end) {
	facing_end = p_facing_end;
}

int EPASWarpPoint::get_rotation_start() const {
	return rotation_start;
}

void EPASWarpPoint::set_rotation_start(int p_rotation_start) {
	rotation_start = p_rotation_start;
}

int EPASWarpPoint::get_rotation_end() const {
	return rotation_end;
}

void EPASWarpPoint::set_rotation_end(int p_rotation_end) {
	rotation_end = p_rotation_end;
}

int EPASWarpPoint::get_translation_start() const {
	return translation_start;
}

void EPASWarpPoint::set_translation_start(int p_translation_start) {
	translation_start = p_translation_start;
}

int EPASWarpPoint::get_translation_end() const {
	return translation_end;
}

void EPASWarpPoint::set_translation_end(int p_translation_end) {
	translation_end = p_translation_end;
}

bool EPASWarpPoint::has_facing() const {
	return facing_start > -1 && facing_end > -1;
}

bool EPASWarpPoint::has_rotation() const {
	return rotation_start > -1 && rotation_end > -1;
}

bool EPASWarpPoint::has_translation() const {
	return translation_start > -1 && translation_end > -1;
}
