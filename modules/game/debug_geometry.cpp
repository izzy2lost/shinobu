#include "debug_geometry.h"
#include "scene/resources/shape_3d.h"

void HBDebugGeometry::clear() {
	im->clear_surfaces();

	for (KeyValue<Ref<Shape3D>, MultiMeshInstance3D *> kv : shape_map) {
		memdelete(kv.value);
	}
	shape_map.clear();
}

void HBDebugGeometry::debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color) {
	im->surface_begin(Mesh::PRIMITIVE_LINES);
	im->surface_set_color(p_color);
	im->surface_add_vertex(p_params.from);
	im->surface_set_color(p_color);
	im->surface_add_vertex(p_params.to);
	im->surface_end();
}

void HBDebugGeometry::debug_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color) {
	ERR_FAIL_COND(!p_shape.is_valid());
	if (!shape_map.has(p_shape)) {
		Ref<MultiMesh> mm;
		mm.instantiate();

		Ref<ArrayMesh> debug_mesh = p_shape->get_debug_mesh()->duplicate(true);
		Ref<StandardMaterial3D> mat = debug_mesh->surface_get_material(0);
		mat->set_albedo(Color(1.0f, 1.0f, 1.0f));

		mm->set_transform_format(MultiMesh::TRANSFORM_3D);
		mm->set_use_colors(true);
		mm->set_mesh(debug_mesh);

		MultiMeshInstance3D *mmi = memnew(MultiMeshInstance3D);

		shape_map.insert(p_shape, mmi);
		add_child(mmi);
	}
	Ref<MultiMesh> mm = shape_map[p_shape]->get_multimesh();
	mm->set_instance_count(mm->get_instance_count() + 1);
	Transform3D trf;
	trf.origin = p_position;
	mm->set_instance_transform(mm->get_instance_count() - 1, trf);
	mm->set_instance_color(mm->get_instance_count() - 1, p_color);
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
