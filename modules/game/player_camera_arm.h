
#ifndef PLAYER_CAMERA_ARM_H
#define PLAYER_CAMERA_ARM_H

#include "modules/game/debug_geometry.h"
#include "modules/game/state_machine.h"
#include "scene/3d/physics/spring_arm_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/3d/shape_3d.h"

class HBPlayerAgent;

class HBPlayerCameraArm : public SpringArm3D {
	GDCLASS(HBPlayerCameraArm, SpringArm3D);

private:
	Vector2 velocity;
	Vector2 acceleration;

	const float max_velocity = 120.0f;
	const float acceleration_degrees = 45.0f;
	const float max_pitch_degrees = 45.0f;
	const float min_pitch_degrees = -65.0f;

	float height_offset = 0.7f;
	float forward_offset = 0.0f;
	float node_tracking_radius = 0.0f;
	float transition_duration = 0.75f;

	Ref<RotationInertializer> rot_inertializer;
	Ref<PositionInertializer> pos_inertializer;
	bool inertialization_queued = false;

	Transform3D prev_transform;
	Transform3D arm_transform;

	float prev_length = 2.0f;
	float length = 2.0f;
	Ref<ScalarInertializer> len_inertializer;

	float target_length = 2.0f;

	void _process_inertialization(float p_delta);
	void _inertialize(Transform3D &p_new_transform, float p_new_len, float p_duration, float p_delta);

public:
	enum CameraTargetMode {
		BONE,
		TRACK_NODES
	};

private:
	CameraTargetMode target_mode = CameraTargetMode::BONE;

	// Mouse moves outside of the velocity system
	Vector2 mouse_target;

	Vector3 position_spring_velocity;

	NodePath target_skeleton_path;
	StringName target_bone_name;
	ObjectID target_skeleton_node_cache;
	void _update_target_skeleton_node_cache();
	Skeleton3D *get_target_skeleton() const;
	Vector3 get_target_position() const;

	HashSet<ObjectID> tracked_nodes;

	HBPlayerAgent *player_parent = nullptr;

	bool tracked_nodes_update_queued = false;

protected:
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;
	void _notification(int p_what);
	static void _bind_methods();

	void _process_rotation(float p_delta);
	void _track_bone(float p_delta);
	void _track_nodes(float p_delta);
	void _on_agent_entered_combat(HBAgent *p_agent);
	void _on_agent_exited_combat(HBAgent *p_agent);

public:
	void clear_tracked_nodes();
	void track_node(Node3D *p_node);

	void set_target_skeleton_path(const NodePath &p_skeleton_path);
	NodePath get_target_skeleton_path() const;
	HBPlayerCameraArm();

	StringName get_target_bone_name() const;
	void set_target_bone_name(const StringName &p_target_bone_name);

	CameraTargetMode get_target_mode() const;
	void set_target_mode(CameraTargetMode p_target_mode);

	float get_node_tracking_radius() const { return node_tracking_radius; }
	void set_node_tracking_radius(float node_tracking_radius_) { node_tracking_radius = node_tracking_radius_; }

	float get_transition_duration() const { return transition_duration; }
	void set_transition_duration(float p_transition_duration) { transition_duration = p_transition_duration; }

	float get_target_length() const { return target_length; }
	void set_target_length(float p_target_length) { target_length = p_target_length; }
};

VARIANT_ENUM_CAST(HBPlayerCameraArm::CameraTargetMode);

#endif // PLAYER_CAMERA_ARM_H
