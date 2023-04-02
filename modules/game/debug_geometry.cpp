#include "debug_geometry.h"

void HBDebugGeometry::clear() {
	im->clear_surfaces();
}

void HBDebugGeometry::debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color) {
	im->surface_begin(Mesh::PRIMITIVE_LINES);
	im->surface_set_color(p_color);
	im->surface_add_vertex(p_params.from);
	im->surface_set_color(p_color);
	im->surface_add_vertex(p_params.to);
	im->surface_end();
}

HBDebugGeometry::HBDebugGeometry() {
	mi = memnew(MeshInstance3D);
	Ref<StandardMaterial3D> mat;
	mat.instantiate();
	mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	mat->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);

	mi->set_material_override(mat);
	im.instantiate();
	mi->set_mesh(im);

	add_child(mi, false, INTERNAL_MODE_BACK);
}
