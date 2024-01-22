#ifndef INERTIALIZATION_H
#define INERTIALIZATION_H
#include "animation_system/epas_pose.h"
#include "core/object/ref_counted.h"

class EPASPoseInertializer;
class RotationInertializer : public RefCounted {
	GDCLASS(RotationInertializer, RefCounted);
	float current_transition_time = 0.0f;
	Vector3 rotation_offset_axis;
	float rotation_offset_angle = 0.0f;
	float rotation_velocity = 0.0f;
	float transition_duration = 0.0f;

protected:
	static void _bind_methods();

public:
	Quaternion advance(float p_delta);
	Quaternion advance_cock(float p_delta);
	bool is_done() const;
	float get_offset_angle() const;
	Vector3 get_offset_axis() const;
	static Ref<RotationInertializer> create(const Quaternion &p_prev_prev, const Quaternion &p_prev, const Quaternion &p_target, float p_duration, float p_delta);
	friend class EPASPoseInertializer;
};

class PositionInertializer : public RefCounted {
	float current_transition_time = 0.0f;
	Vector3 position_offset;
	float position_velocity = 0.0f;
	float transition_duration = 0.0f;

public:
	Vector3 advance(float p_delta);
	bool is_done() const;
	Vector3 get_offset() const;
	static Ref<PositionInertializer> create(const Vector3 &p_prev_prev, const Vector3 &p_prev, const Vector3 &p_target, float p_duration, float p_delta);
};

class EPASPoseInertializer : public RefCounted {
	struct TransitionInfo {
		StringName bone_name;
		Ref<PositionInertializer> position_inertializer;
		Ref<RotationInertializer> rotation_inertializer;
	};
	Vector<TransitionInfo> transition_infos;
	float current_transition_time = 0.0f;

public:
	enum InertializationPose {
		PREV_PREV_POSE = 0,
		PREV_POSE = 1,
		TARGET_POSE = 2,
		POSE_MAX = 3
	};
	float get_current_transition_time() const;
	bool advance(Ref<EPASPose> p_target_pose, const Ref<EPASPose> &p_base_pose, float p_delta);
	static Ref<EPASPoseInertializer> create(const Ref<EPASPose> p_poses[POSE_MAX], const Ref<EPASPose> &p_base_pose, float p_transition_duration, float p_delta, TypedArray<StringName> p_bone_filter = TypedArray<StringName>());
};

#endif // INERTIALIZATION_H
