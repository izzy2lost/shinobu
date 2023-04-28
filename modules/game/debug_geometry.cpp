#include "debug_geometry.h"
#include "scene/resources/primitive_meshes.h"
#include "scene/resources/shape_3d.h"

Ref<StandardMaterial3D> HBDebugGeometry::debug_material;

Ref<StandardMaterial3D> HBDebugGeometry::get_debug_material() {
	if (!debug_material.is_valid()) {
		debug_material.instantiate();
		debug_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
		debug_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		debug_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
	}
	return debug_material;
}

void HBDebugGeometry::clear() {
	im->clear_surfaces();
	if (sphere_multi_mesh) {
		sphere_multi_mesh->get_multimesh()->set_visible_instance_count(0);
		curr_sphere_instance = 0;
	}

	for (KeyValue<Ref<Shape3D>, MultiMeshInstance3D *> kv : shape_map) {
		memdelete(kv.value);
	}
	shape_map.clear();
}

void HBDebugGeometry::debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color) {
	im->surface_begin(Mesh::PRIMITIVE_LINES);
	im->surface_set_color(p_color);
	im->surface_add_vertex(p_params.from);
	im->surface_add_vertex(p_params.to);

	Vector3 dir = p_params.from.direction_to(p_params.to);

	Vector3 normal = (fabs(dir.x) + fabs(dir.y) > CMP_EPSILON) ? Vector3(-dir.y, dir.x, 0).normalized() : Vector3(0, -dir.z, dir.y).normalized();

	for (int i = 0; i < 4; i++) {
		float prog = i / 4.0f;
		float prog_plus_one = (i + 1) / 4.0f;
		Vector3 v_curr = p_params.to + normal.rotated(dir, prog * Math_TAU) * 0.1f;
		Vector3 v_next = p_params.to + normal.rotated(dir, prog_plus_one * Math_TAU) * 0.1f;
		v_curr -= dir * 0.1f;
		v_next -= dir * 0.1f;
		im->surface_add_vertex(v_curr);
		im->surface_add_vertex(v_next);
		im->surface_add_vertex(p_params.to);
		im->surface_add_vertex(v_curr);
	}

	im->surface_end();
}

void HBDebugGeometry::debug_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color) {
	im->surface_begin(Mesh::PRIMITIVE_LINES);
	im->surface_set_color(p_color);
	im->surface_add_vertex(p_from);
	im->surface_add_vertex(p_to);
	im->surface_end();
}

void HBDebugGeometry::debug_shape(Ref<Shape3D> p_shape, const Vector3 &p_position, const Color &p_color) {
	ERR_FAIL_COND(!p_shape.is_valid());
	if (!shape_map.has(p_shape)) {
		Ref<MultiMesh> mm;
		mm.instantiate();

		Ref<ArrayMesh> debug_mesh = p_shape->get_debug_mesh();

		mm->set_transform_format(MultiMesh::TRANSFORM_3D);
		mm->set_use_colors(true);
		mm->set_mesh(debug_mesh);

		MultiMeshInstance3D *mmi = memnew(MultiMeshInstance3D);
		mmi->set_material_override(get_debug_material());
		mmi->set_multimesh(mm);

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

void HBDebugGeometry::debug_sphere(const Vector3 &p_position, float p_radius, const Color &p_color) {
	if (!sphere_multi_mesh) {
		sphere_multi_mesh = memnew(MultiMeshInstance3D);
		add_child(sphere_multi_mesh);
		Ref<MultiMesh> multi_mesh;
		multi_mesh.instantiate();
		Ref<SphereMesh> sm;
		sm.instantiate();
		sm->set_radius(1.0f);
		sm->set_height(2.0f);
		multi_mesh->set_mesh(sm);
		multi_mesh->set_use_colors(true);
		multi_mesh->set_transform_format(MultiMesh::TRANSFORM_3D);
		multi_mesh->set_instance_count(SPHERE_INSTANCE_COUNT);
		multi_mesh->set_visible_instance_count(0);
		sphere_multi_mesh->set_multimesh(multi_mesh);
		sphere_multi_mesh->set_material_override(get_debug_material());
	}
	Ref<MultiMesh> mm = sphere_multi_mesh->get_multimesh();

	mm->set_visible_instance_count(MIN(curr_sphere_instance + 1, SPHERE_INSTANCE_COUNT));
	int mm_idx = curr_sphere_instance;

	Transform3D trf;
	trf.origin = p_position;
	trf.basis.scale(Vector3(1.0f, 1.0f, 1.0f) * p_radius);
	mm->set_instance_transform(mm_idx, trf);
	mm->set_instance_color(mm_idx, p_color);
	curr_sphere_instance = (curr_sphere_instance + 1) % SPHERE_INSTANCE_COUNT;
}

HBDebugGeometry::HBDebugGeometry() {
	MeshInstance3D *mi;
	mi = memnew(MeshInstance3D);
	mi->set_material_override(get_debug_material());
	im.instantiate();
	mi->set_mesh(im);

	add_child(mi, false, INTERNAL_MODE_BACK);
}
