#ifndef DEBUG_GEOMETRY_H
#define DEBUG_GEOMETRY_H

#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/immediate_mesh.h"
#include "servers/physics_server_3d.h"

class HBDebugGeometry : public Node3D {
	GDCLASS(HBDebugGeometry, Node3D);

private:
	MeshInstance3D *mi;
	Ref<ImmediateMesh> im;

public:
	void clear();
	void debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color());
	HBDebugGeometry();
};

#endif // DEBUG_GEOMETRY_H
