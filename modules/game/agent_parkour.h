#ifndef AGENT_PARKOUR_H
#define AGENT_PARKOUR_H

#include "agent.h"
#include "scene/3d/collision_shape_3d.h"
#include "scene/3d/physics_body_3d.h"
#include "scene/resources/box_shape_3d.h"
#include "scene/resources/sphere_shape_3d.h"

class HBAgentParkourPoint : public StaticBody3D {
	GDCLASS(HBAgentParkourPoint, StaticBody3D);
	bool collision_shape_dirty = true;
	Ref<SphereShape3D> shape;
	CollisionShape3D *collision_shape = nullptr;
	void _update_collision_shape();

protected:
	void _notification(int p_what);

public:
	HBAgentParkourPoint();
};

#endif // AGENT_PARKOUR_H
