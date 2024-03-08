
#ifndef PLAYER_CAMERA_ARM_H
#define PLAYER_CAMERA_ARM_H

#include "modules/game/debug_geometry.h"
#include "modules/game/state_machine.h"
#include "scene/3d/physics/spring_arm_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/3d/shape_3d.h"

class HBPlayerCameraArm : public SpringArm3D {
	GDCLASS(HBPlayerCameraArm, SpringArm3D);

private:
	Vector2 velocity;
	Vector2 acceleration;

	const float max_velocity = 120.0f;
	const float acceleration_degrees = 45.0f;
	const float max_pitch_degrees = 45.0f;
	const float min_pitch_degrees = -65.0f;
	float height_offset = 0.0f;
	float forward_offset = 5.0f;
	HBDebugGeometry *debug_geo = nullptr;

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

	LocalVector<ObjectID> tracked_nodes;

protected:
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;
	void _notification(int p_what);
	static void _bind_methods();

	void _process_rotation(float p_delta);
	void _track_bone(float p_delta);
	void _track_nodes(float p_delta);

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
};

VARIANT_ENUM_CAST(HBPlayerCameraArm::CameraTargetMode);

#endif // PLAYER_CAMERA_ARM_H
