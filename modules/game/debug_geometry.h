#ifndef DEBUG_GEOMETRY_H
#define DEBUG_GEOMETRY_H

#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/multimesh_instance_3d.h"
#include "scene/3d/node_3d.h"
#include "scene/resources/immediate_mesh.h"
#include "servers/physics_server_3d.h"

class HBDebugGeometry : public Node3D {
	GDCLASS(HBDebugGeometry, Node3D);

private:
	const int SPHERE_INSTANCE_COUNT = 10;
	int curr_sphere_instance = 0;
	Ref<ImmediateMesh> im;
	HashMap<Ref<Shape3D>, MultiMeshInstance3D *> shape_map;
	MultiMeshInstance3D *sphere_multi_mesh = nullptr;
	static Ref<StandardMaterial3D> debug_material;
	static Ref<StandardMaterial3D> get_debug_material();

public:
	void clear();
	void debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color());
	void debug_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color());
	void debug_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color = Color());
	void debug_sphere(const Vector3 &p_position, float p_radius = 0.05f, const Color &p_color = Color());
	HBDebugGeometry();
};

#endif // DEBUG_GEOMETRY_H
