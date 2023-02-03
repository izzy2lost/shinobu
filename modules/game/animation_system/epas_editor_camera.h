#ifndef EPAS_EDITOR_CAMERA_H
#define EPAS_EDITOR_CAMERA_H

#include "scene/3d/camera_3d.h"

class EPASEditorCamera : public Camera3D {
	GDCLASS(EPASEditorCamera, Camera3D);

protected:
	void _notification(int p_what);
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;

public:
	EPASEditorCamera();
};

#endif // EPAS_EDITOR_CAMERA_H