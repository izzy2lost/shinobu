
#ifndef PLAYER_CAMERA_ARM_H
#define PLAYER_CAMERA_ARM_H

#include "scene/3d/skeleton_3d.h"
#include "scene/3d/spring_arm_3d.h"
#include "scene/resources/shape_3d.h"

class HBPlayerCameraArm : public SpringArm3D {
	GDCLASS(HBPlayerCameraArm, SpringArm3D);

private:
	Vector2 velocity;
	Vector2 acceleration;

	const float max_velocity = 120.0f;
	const float acceleration_degrees = 45.0f;
	const float max_pitch_degrees = 45.0f;
	const float min_pitch_degrees = -65.0f;

	// Mouse moves outside of the velocity system
	Vector2 mouse_target;

	Vector3 position_spring_velocity;

	NodePath target_skeleton_path;
	StringName target_bone_name;
	ObjectID target_skeleton_node_cache;
	void _update_target_skeleton_node_cache();
	Skeleton3D *get_target_skeleton() const;
	Vector3 get_target_position() const;

protected:
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_target_skeleton_path(const NodePath &p_skeleton_path);
	NodePath get_target_skeleton_path() const;
	HBPlayerCameraArm();

	StringName get_target_bone_name() const;
	void set_target_bone_name(const StringName &p_target_bone_name);
};

#endif // PLAYER_CAMERA_ARM_H
