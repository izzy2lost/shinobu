/**************************************************************************/
/*  debug_geometry.h                                                      */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

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
	const int SPHERE_INSTANCE_COUNT = 32;
	Ref<StandardMaterial3D> debug_material;
	Ref<StandardMaterial3D> get_debug_material();
	struct DebugGeometryGroup {
		int curr_sphere_instance = 0;
		MeshInstance3D *immediate_mesh_instance;
		Ref<ImmediateMesh> immediate_mesh;
		HashMap<Ref<Shape3D>, MultiMeshInstance3D *> shape_map;
		MultiMeshInstance3D *sphere_multi_mesh = nullptr;
		StringName group_name;
		bool visible = true;
	};

	Vector<DebugGeometryGroup> groups;

	StringName current_group_name = "default";
	DebugGeometryGroup *current_group = nullptr;

	void _draw_arrow(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color());

protected:
	static void _bind_methods();

public:
	void add_group(const StringName &p_group_name);
	void set_current_group(const StringName &p_group_name);
	void clear();
	void debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color = Color());
	void debug_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color = Color());
	void debug_shape(Ref<Shape3D> p_shape, const Transform3D &p_trf, const Color &p_color = Color());
	void debug_sphere(const Vector3 &p_position, float p_radius = 0.05f, const Color &p_color = Color());
	void debug_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color = Color());
	void set_group_visible(const StringName &p_group_name, bool p_visible);
	bool get_group_visible(const StringName &p_group_name) const;
	int get_group_count() const;
	StringName get_group_name(int p_idx) const;
	HBDebugGeometry();
};

#endif // DEBUG_GEOMETRY_H
