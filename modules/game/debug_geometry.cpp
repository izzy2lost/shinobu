#include "debug_geometry.h"
#include "scene/resources/primitive_meshes.h"
#include "scene/resources/shape_3d.h"

Ref<StandardMaterial3D> HBDebugGeometry::get_debug_material() {
	if (debug_material.is_valid()) {
		return debug_material;
	}

	debug_material.instantiate();
	Ref<StandardMaterial3D> mat;
	mat.instantiate();
	mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	mat->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
	debug_material = mat;
	return debug_material;
}

void HBDebugGeometry::_draw_arrow(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color) {
	if (p_from == p_to) {
		return;
	}
	current_group->immediate_mesh->surface_set_color(p_color);
	current_group->immediate_mesh->surface_add_vertex(p_from);
	current_group->immediate_mesh->surface_add_vertex(p_to);

	Vector3 dir = p_from.direction_to(p_to);

	Vector3 normal = (fabs(dir.x) + fabs(dir.y) > CMP_EPSILON) ? Vector3(-dir.y, dir.x, 0).normalized() : Vector3(0, -dir.z, dir.y).normalized();

	for (int i = 0; i < 4; i++) {
		float prog = i / 4.0f;
		float prog_plus_one = (i + 1) / 4.0f;
		Vector3 v_curr = p_to + normal.rotated(dir, prog * Math_TAU) * 0.1f;
		Vector3 v_next = p_to + normal.rotated(dir, prog_plus_one * Math_TAU) * 0.1f;
		v_curr -= dir * 0.1f;
		v_next -= dir * 0.1f;
		current_group->immediate_mesh->surface_add_vertex(v_curr);
		current_group->immediate_mesh->surface_add_vertex(v_next);
		current_group->immediate_mesh->surface_add_vertex(p_to);
		current_group->immediate_mesh->surface_add_vertex(v_curr);
	}
}

void HBDebugGeometry::_bind_methods() {
	ClassDB::bind_method(D_METHOD("clear"), &HBDebugGeometry::clear);
	ClassDB::bind_method(D_METHOD("add_group", "group_name"), &HBDebugGeometry::add_group);
	ClassDB::bind_method(D_METHOD("set_current_group", "group_name"), &HBDebugGeometry::set_current_group);
	ClassDB::bind_method(D_METHOD("debug_line", "from", "to", "color"), &HBDebugGeometry::debug_line);
	ClassDB::bind_method(D_METHOD("debug_sphere", "position", "radius", "color"), &HBDebugGeometry::debug_sphere);
	ClassDB::bind_method(D_METHOD("set_group_visible", "group_name", "visible"), &HBDebugGeometry::set_group_visible);
	ClassDB::bind_method(D_METHOD("get_group_visible", "group_name"), &HBDebugGeometry::get_group_visible);
}

void HBDebugGeometry::add_group(const StringName &p_group_name) {
	for (int i = 0; i < groups.size(); i++) {
		ERR_FAIL_COND_MSG(groups[i].group_name == p_group_name, vformat("Group %s already exists!", p_group_name));
	}
	DebugGeometryGroup group;
	group.immediate_mesh_instance = memnew(MeshInstance3D);
	group.immediate_mesh_instance->set_material_override(get_debug_material());
	group.immediate_mesh.instantiate();
	group.immediate_mesh_instance->set_mesh(group.immediate_mesh);

	add_child(group.immediate_mesh_instance, false, INTERNAL_MODE_BACK);
	group.group_name = p_group_name;

	groups.push_back(group);

	// Group likely changed address because we resized the group vector
	set_current_group(current_group_name);
}

void HBDebugGeometry::set_current_group(const StringName &p_group_name) {
	DebugGeometryGroup *group = nullptr;
	for (int i = 0; i < groups.size(); i++) {
		group = &groups.write[i];
	}
	ERR_FAIL_COND_MSG(!group, vformat("Group %s does not exist!", p_group_name));

	current_group_name = p_group_name;
	current_group = group;
}

void HBDebugGeometry::clear() {
	current_group->immediate_mesh->clear_surfaces();
	if (current_group->sphere_multi_mesh) {
		current_group->sphere_multi_mesh->get_multimesh()->set_visible_instance_count(0);
		current_group->curr_sphere_instance = 0;
	}

	for (KeyValue<Ref<Shape3D>, MultiMeshInstance3D *> kv : current_group->shape_map) {
		memdelete(kv.value);
	}
	current_group->shape_map.clear();
}

void HBDebugGeometry::debug_raycast(const PhysicsDirectSpaceState3D::RayParameters &p_params, const Color &p_color) {
	current_group->immediate_mesh->surface_begin(Mesh::PRIMITIVE_LINES);

	_draw_arrow(p_params.from, p_params.to, p_color);

	current_group->immediate_mesh->surface_end();
}

void HBDebugGeometry::debug_line(const Vector3 &p_from, const Vector3 &p_to, const Color &p_color) {
	current_group->immediate_mesh->surface_begin(Mesh::PRIMITIVE_LINES);
	current_group->immediate_mesh->surface_set_color(p_color);
	current_group->immediate_mesh->surface_add_vertex(p_from);
	current_group->immediate_mesh->surface_add_vertex(p_to);
	current_group->immediate_mesh->surface_end();
}

void HBDebugGeometry::debug_shape(Ref<Shape3D> p_shape, const Transform3D &p_trf, const Color &p_color) {
	ERR_FAIL_COND(!p_shape.is_valid());
	if (!current_group->shape_map.has(p_shape)) {
		Ref<MultiMesh> mm;
		mm.instantiate();

		Ref<ArrayMesh> debug_mesh = p_shape->get_debug_mesh();

		mm->set_transform_format(MultiMesh::TRANSFORM_3D);
		mm->set_use_colors(true);
		mm->set_mesh(debug_mesh);

		MultiMeshInstance3D *mmi = memnew(MultiMeshInstance3D);
		mmi->set_material_override(get_debug_material());
		mmi->set_multimesh(mm);

		mm->set_instance_count(SPHERE_INSTANCE_COUNT);
		mm->set_visible_instance_count(0);

		current_group->shape_map.insert(p_shape, mmi);
		add_child(mmi);
	}
	Ref<MultiMesh> mm = current_group->shape_map[p_shape]->get_multimesh();
	mm->set_visible_instance_count(mm->get_visible_instance_count() + 1);
	mm->set_instance_transform(mm->get_visible_instance_count() - 1, p_trf);
	mm->set_instance_color(mm->get_visible_instance_count() - 1, p_color);
}

void HBDebugGeometry::debug_sphere(const Vector3 &p_position, float p_radius, const Color &p_color) {
	if (!current_group->sphere_multi_mesh) {
		current_group->sphere_multi_mesh = memnew(MultiMeshInstance3D);
		current_group->sphere_multi_mesh->set_visible(current_group->visible);
		add_child(current_group->sphere_multi_mesh);
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
		current_group->sphere_multi_mesh->set_multimesh(multi_mesh);
		print_line("DEBUG MAT", get_debug_material());
		current_group->sphere_multi_mesh->set_material_override(get_debug_material());
	}
	Ref<MultiMesh> mm = current_group->sphere_multi_mesh->get_multimesh();

	mm->set_visible_instance_count(MIN(current_group->curr_sphere_instance + 1, SPHERE_INSTANCE_COUNT));
	int mm_idx = current_group->curr_sphere_instance;

	Transform3D trf;
	trf.origin = p_position;
	trf.basis.scale(Vector3(1.0f, 1.0f, 1.0f) * p_radius);
	mm->set_instance_transform(mm_idx, trf);
	mm->set_instance_color(mm_idx, p_color);
	current_group->curr_sphere_instance = (current_group->curr_sphere_instance + 1) % SPHERE_INSTANCE_COUNT;
}

void HBDebugGeometry::debug_cast_motion(const Ref<Shape3D> &p_shape, const PhysicsDirectSpaceState3D::ShapeParameters &p_shape_cast_3d, const Color &p_color) {
	if (!p_shape_cast_3d.motion.is_zero_approx()) {
		current_group->immediate_mesh->surface_begin(Mesh::PRIMITIVE_LINES);
		_draw_arrow(p_shape_cast_3d.transform.origin, p_shape_cast_3d.transform.origin + p_shape_cast_3d.motion, p_color);
		current_group->immediate_mesh->surface_end();
	}
	debug_shape(p_shape, p_shape_cast_3d.transform, p_color);
}

void HBDebugGeometry::set_group_visible(const StringName &p_group_name, bool p_visible) {
	DebugGeometryGroup *group = nullptr;
	for (int i = 0; i < groups.size(); i++) {
		group = &groups.write[i];
	}

	ERR_FAIL_COND_MSG(!group, vformat("Group %s does not exist!", p_group_name));

	group->visible = p_visible;
	group->immediate_mesh_instance->set_visible(p_visible);
	if (group->sphere_multi_mesh) {
		group->sphere_multi_mesh->set_visible(p_visible);
	}
	for (KeyValue<Ref<Shape3D>, MultiMeshInstance3D *> kv : group->shape_map) {
		kv.value->set_visible(p_visible);
	}
}

bool HBDebugGeometry::get_group_visible(const StringName &p_group_name) const {
	const DebugGeometryGroup *group = nullptr;
	for (int i = 0; i < groups.size(); i++) {
		group = &groups[i];
	}

	ERR_FAIL_COND_V_MSG(!group, false, vformat("Group %s does not exist!", p_group_name));
	return group->visible;
}

int HBDebugGeometry::get_group_count() const {
	return groups.size();
}

StringName HBDebugGeometry::get_group_name(int p_idx) const {
	ERR_FAIL_INDEX_V(p_idx, groups.size(), "");
	return groups[p_idx].group_name;
}

HBDebugGeometry::HBDebugGeometry() {
	current_group_name = "default";
	add_group("default");
}
