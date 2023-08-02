#include "agent_parkour.h"
#include "physics_layers.h"

#include "scene/resources/sphere_shape_3d.h"

void HBAgentParkourPoint::_update_collision_shape() {
	collision_shape_dirty = false;
	if (!collision_shape) {
		collision_shape = memnew(CollisionShape3D);
		shape.instantiate();
		add_child(collision_shape, false, INTERNAL_MODE_BACK);
	}
	shape->set_radius(0.1f);
	collision_shape->set_shape(shape);
}

void HBAgentParkourPoint::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PROCESS: {
			if (collision_shape_dirty) {
				_update_collision_shape();
				set_process(false);
			}
		} break;
	}
}

HBAgentParkourPoint::HBAgentParkourPoint() {
	set_collision_mask(0);
	set_collision_layer(HBPhysicsLayers::LAYER_PARKOUR_NODES);
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_process(true);
	}
}
void HBAgentParkourBeam::_editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) {
	if (p_info.parents.size() > 0) {
		queue_free();
		return;
	}
	Vector<StringName> children = p_info.children;
	curve.instantiate();
	curve->add_point(Vector3());
	while (children.size() > 0) {
		StringName child = children[0];
		const EntityCompileInfo *child_info = p_entities.getptr(child);
		if (!child_info) {
			break;
		}

		curve->add_point(to_local(child_info->node->get_global_position()));

		children = child_info->children;

		CollisionShape3D *col_shape = memnew(CollisionShape3D);
		Ref<BoxShape3D> box_shape;
		box_shape.instantiate();

		const float COLLISION_RADIUS = 0.1f;
		Vector3 p1 = curve->get_point_position(curve->get_point_count() - 1);
		Vector3 p2 = curve->get_point_position(curve->get_point_count() - 2);
		box_shape->set_size(Vector3(COLLISION_RADIUS, COLLISION_RADIUS, p1.distance_to(p2)));
		col_shape->set_shape(box_shape);
		add_child(col_shape);
		col_shape->set_position((p1 + p2) * 0.5f);
		Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), col_shape->get_position().direction_to(p2));
		col_shape->set_quaternion(rot);
		col_shape->set_owner(get_tree()->get_edited_scene_root());
	}
}

void HBAgentParkourBeam::_line_redraw() {
	Ref<Curve3D> c = get_curve();
	if (c.is_null()) {
		return;
	}

	real_t interval = 0.1;
	const real_t length = c->get_baked_length();

	Ref<StandardMaterial3D> mat;
	mat.instantiate();
	mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
	mat->set_albedo(Color(1.0f, 0.0f, 0.0f));

	Array mesh_data;
	mesh_data.resize(ArrayMesh::ARRAY_MAX);

	PackedVector3Array v3p;

	// 1. Draw curve and bones.
	if (length > CMP_EPSILON) {
		const int sample_count = int(length / interval) + 2;
		interval = length / (sample_count - 1); // Recalculate real interval length.

		Vector<Transform3D> frames;
		frames.resize(sample_count);

		{
			Transform3D *w = frames.ptrw();

			for (int i = 0; i < sample_count; i++) {
				w[i] = c->sample_baked_with_rotation(i * interval, true, true);
			}
		}

		const Transform3D *r = frames.ptr();
		for (int i = 0; i < sample_count - 1; i++) {
			const Vector3 p1 = r[i].origin;
			const Vector3 p2 = r[i + 1].origin;
			const Vector3 side = r[i].basis.get_column(0);
			const Vector3 up = r[i].basis.get_column(1);
			const Vector3 forward = r[i].basis.get_column(2);

			// Curve segment.
			v3p.push_back(p1);
			v3p.push_back(p2);

			// Fish Bone.
			v3p.push_back(p1);
			v3p.push_back(p1 + (side + forward + up * 0.3) * 0.06);

			v3p.push_back(p1);
			v3p.push_back(p1 + (-side + forward + up * 0.3) * 0.06);
		}
	}

	Ref<ArrayMesh> arr_mesh;
	arr_mesh.instantiate();
	mesh_data[ArrayMesh::ARRAY_VERTEX] = v3p;
	arr_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, mesh_data);
	arr_mesh->surface_set_material(0, mat);

	if (!debug_preview) {
		debug_preview = memnew(MeshInstance3D);
		add_child(debug_preview);
		debug_preview->set_mesh(arr_mesh);
	}
}

void HBAgentParkourBeam::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_curve", "curve"), &HBAgentParkourBeam::set_curve);
	ClassDB::bind_method(D_METHOD("get_curve"), &HBAgentParkourBeam::get_curve);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve3D"), "set_curve", "get_curve");
}

void HBAgentParkourBeam::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			_queue_line_redraw();
		} break;
	}
}

void HBAgentParkourBeam::set_curve(Ref<Curve3D> p_curve) {
	curve = p_curve;
}

Ref<Curve3D> HBAgentParkourBeam::get_curve() const {
	return curve;
}

void HBAgentParkourBeam::_queue_line_redraw() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (line_redraw_queued) {
		return;
	}
	line_redraw_queued = true;
	callable_mp(this, &HBAgentParkourBeam::_line_redraw).call_deferred();
}

HBAgentParkourBeam::HBAgentParkourBeam() {
	set_collision_layer(HBPhysicsLayers::LAYER_PARKOUR_NODES);
	set_collision_mask(0);
}
