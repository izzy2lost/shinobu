
#ifndef PLAYER_CAMERA_ARM_H
#define PLAYER_CAMERA_ARM_H

#include "core/config/project_settings.h"
#include "scene/3d/spring_arm_3d.h"
#include "scene/resources/shape_3d.h"

class HBPlayerCameraArm : public SpringArm3D {
	GDCLASS(HBPlayerCameraArm, SpringArm3D);

private:
	Vector2 velocity;

	// Mouse moves outside of the velocity system
	Vector2 mouse_target;

protected:
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;
	void _notification(int p_what);
	HBPlayerCameraArm();
};

#endif // PLAYER_CAMERA_ARM_H
