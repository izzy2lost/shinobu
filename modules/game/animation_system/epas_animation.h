#ifndef EPAS_ANIMATION_H
#define EPAS_ANIMATION_H

#include "core/io/resource.h"
#include "core/variant/binder_common.h"
#include "core/variant/typed_array.h"
#include "modules/game/animation_system/epas_pose.h"

class EPASKeyframe : public Resource {
	GDCLASS(EPASKeyframe, Resource);
	Ref<EPASPose> pose;
	float time = 0.0f;

protected:
	static void _bind_methods();

public:
	Ref<EPASPose> get_pose() const;
	void set_pose(const Ref<EPASPose> &p_pose);

	float get_time() const;
	void set_time(float p_time);
};

struct EPASAnimationPlaybackInfo {
public:
	bool use_root_motion = false;
	StringName root_bone;
	Transform3D root_motion_trf; // output trf
	Transform3D starting_global_trf; // the starting trf of the skeleton
	HashMap<StringName, Transform3D> warp_point_transforms;
	Vector3 forward = Vector3(0, 0, -1.0f);
};

struct EPASKeyframeComparator {
	_FORCE_INLINE_ bool operator()(const Ref<EPASKeyframe> &a, const Ref<EPASKeyframe> &b) const { return (a->get_time() < b->get_time()); }
};
class EPASWarpPoint : public Resource {
	GDCLASS(EPASWarpPoint, Resource);
	StringName point_name;
	Transform3D transform;
	int facing_start = -1;
	int facing_end = -1;
	int rotation_start = -1;
	int rotation_end = -1;
	int translation_start = -1;
	int translation_end = -1;

protected:
	static void _bind_methods();

public:
	Transform3D get_transform() const;
	void set_transform(const Transform3D &p_transform);

	StringName get_point_name() const;
	void set_point_name(const StringName &p_point_name);

	int get_facing_start() const;
	void set_facing_start(int p_facing_start);

	int get_facing_end() const;
	void set_facing_end(int p_facing_end);

	int get_rotation_start() const;
	void set_rotation_start(int p_rotation_start);

	int get_rotation_end() const;
	void set_rotation_end(int p_rotation_end);

	int get_translation_start() const;
	void set_translation_start(int p_translation_start);

	int get_translation_end() const;
	void set_translation_end(int p_translation_end);

	bool has_facing() const;
	bool has_rotation() const;
	bool has_translation() const;
};
class EPASAnimation : public Resource {
	GDCLASS(EPASAnimation, Resource);

	float length_cache = 0.0f;

	Vector<Ref<EPASKeyframe>> keyframes;
	bool keyframe_order_dirty = true;
	bool length_cache_dirty = true;

	void _sort_keyframes();
	void _update_length_cache();
	void _set_keyframes(const Array &p_keyframes);
	Array _get_keyframes() const;
	void _set_warp_points(const Array &p_warp_points);
	Array _get_warp_points() const;
	void _keyframe_time_changed();

	Vector<Ref<EPASWarpPoint>> warp_points;

protected:
	static void _bind_methods();

public:
	enum InterpolationMethod {
		STEP = 0,
		LINEAR,
		BICUBIC_SPLINE, // Loop only, might break on other things?
		BICUBIC_SPLINE_CLAMPED // Used outside loops
	};
	void add_keyframe(Ref<EPASKeyframe> p_keyframe);
	void erase_keyframe(Ref<EPASKeyframe> p_keyframe);
	int get_keyframe_count() const;
	Ref<EPASKeyframe> get_keyframe(int p_idx) const;
	void interpolate(float p_time, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, InterpolationMethod p_interp_method, EPASAnimationPlaybackInfo *p_playback_info = nullptr) const;
	float get_length() const;
	bool has_warp_point(const StringName &p_name) const;
	void add_warp_point(Ref<EPASWarpPoint> p_warp_point);
	void erase_warp_point(Ref<EPASWarpPoint> p_warp_point);
	int get_warp_point_count() const;
	Ref<EPASWarpPoint> get_warp_point(int p_idx) const;
	void clear_keyframes();
};

VARIANT_ENUM_CAST(EPASAnimation::InterpolationMethod);

#endif // EPAS_ANIMATION_H
