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

struct EPASKeyframeComparator {
	_FORCE_INLINE_ bool operator()(const Ref<EPASKeyframe> &a, const Ref<EPASKeyframe> &b) const { return (a->get_time() < b->get_time()); }
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
	void _keyframe_time_changed();

protected:
	static void _bind_methods();

public:
	enum InterpolationMethod {
		STEP = 0,
		LINEAR,
		BICUBIC_SPLINE // Loop only, might break on other things?
	};

	void add_keyframe(Ref<EPASKeyframe> p_keyframe);
	void erase_keyframe(Ref<EPASKeyframe> p_keyframe);
	int get_keyframe_count() const;
	Ref<EPASKeyframe> get_keyframe(int p_idx) const;
	void interpolate(float p_time, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, InterpolationMethod p_interp_method) const;
	float get_length() const;
	void clear();
};

VARIANT_ENUM_CAST(EPASAnimation::InterpolationMethod);

#endif // EPAS_ANIMATION_H
