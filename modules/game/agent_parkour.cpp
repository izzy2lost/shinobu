/**************************************************************************/
/*  agent_parkour.cpp                                                     */
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

#include "agent_parkour.h"
#include "clipper2/clipper.h"
#include "modules/game/console_system.h"
#include "physics_layers.h"

#include "scene/resources/3d/box_shape_3d.h"
#include "scene/resources/3d/sphere_shape_3d.h"

CVar parkour_debug_enabled = CVar("parkour_debug", Variant::BOOL, false);

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
	beam_points_data.push_back(BeamPointData{ is_edge, Vector3() });

	while (children.size() > 0) {
		StringName child = children[0];
		const EntityCompileInfo *child_info = p_entities.getptr(child);
		if (!child_info) {
			break;
		}

		HBAgentParkourBeam *child_beam_point = Object::cast_to<HBAgentParkourBeam>(child_info->node);

		DEV_ASSERT(child_beam_point != nullptr);

		BeamPointData point_data = {};

		if (child_beam_point) {
			point_data.is_edge = child_beam_point->get_is_edge();
			point_data.position = to_local(child_info->node->get_global_position());
		}

		beam_points_data.push_back(point_data);

		children = child_info->children;

		CollisionShape3D *col_shape = memnew(CollisionShape3D);
		Ref<BoxShape3D> box_shape;
		box_shape.instantiate();

		const float COLLISION_RADIUS = 0.1f;
		Vector3 p1 = beam_points_data[beam_points_data.size() - 1].position;
		Vector3 p2 = beam_points_data[beam_points_data.size() - 2].position;
		box_shape->set_size(Vector3(COLLISION_RADIUS, COLLISION_RADIUS, p1.distance_to(p2)));
		col_shape->set_shape(box_shape);
		add_child(col_shape);
		col_shape->set_position((p1 + p2) * 0.5f);
		Quaternion rot = Quaternion(Vector3(0.0f, 0.0f, -1.0f), col_shape->get_position().direction_to(p2));
		col_shape->set_quaternion(rot);
		col_shape->set_owner(get_tree()->get_edited_scene_root());
	}
}

Ref<Curve3D> HBAgentParkourBeam::get_curve() const {
	if (is_curve_dirty) {
		const_cast<HBAgentParkourBeam *>(this)->_update_curve();
	}
	return curve_cache;
}

void HBAgentParkourBeam::_update_curve() {
	is_curve_dirty = false;
	curve_cache.instantiate();
	for (int i = 0; i < beam_points_data.size(); i++) {
		curve_cache->add_point(beam_points_data[i].position);
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
	ClassDB::bind_method(D_METHOD("get_curve"), &HBAgentParkourBeam::get_curve);

	ClassDB::bind_method(D_METHOD("set_is_edge", "is_edge"), &HBAgentParkourBeam::set_is_edge);
	ClassDB::bind_method(D_METHOD("get_is_edge"), &HBAgentParkourBeam::get_is_edge);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_edge"), "set_is_edge", "get_is_edge");
}

void HBAgentParkourBeam::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			_queue_line_redraw();
		} break;
	}
}

bool HBAgentParkourBeam::_set(const StringName &p_name, const Variant &p_value) {
	if (p_name == SNAME("beam_points_data")) {
		Array bpi = p_value;
		beam_points_data.resize_zeroed(bpi.size());
		BeamPointData *bpi_w = beam_points_data.ptrw();
		for (int i = 0; i < bpi.size(); i++) {
			Dictionary point_data = bpi[i];
			bpi_w[i].is_edge = point_data.get("is_edge", false);
			bpi_w[i].position = point_data.get("position", Vector3());
		}
		return true;
	}
	return false;
}

bool HBAgentParkourBeam::_get(const StringName &p_name, Variant &r_ret) const {
	if (p_name == SNAME("beam_points_data")) {
		Array bpi;
		for (int i = 0; i < beam_points_data.size(); i++) {
			Dictionary point_data;
			point_data["is_edge"] = beam_points_data[i].is_edge;
			point_data["position"] = beam_points_data[i].position;
			bpi.push_back(point_data);
		}
		r_ret = bpi;
		return true;
	}
	return false;
}

void HBAgentParkourBeam::_get_property_list(List<PropertyInfo> *p_list) const {
	p_list->push_back(PropertyInfo(Variant::ARRAY, "beam_points_data", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL));
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

bool HBAgentParkourBeam::get_is_edge() const { return is_edge; }

void HBAgentParkourBeam::set_is_edge(bool p_is_edge) { is_edge = p_is_edge; }

bool HBAgentParkourBeam::is_point_edge(int p_point_idx) const {
	ERR_FAIL_INDEX_V(p_point_idx, beam_points_data.size(), false);
	return beam_points_data[p_point_idx].is_edge;
}

void HBAgentParkourLedge::_on_parkour_debug_changed() {
	bool visible = parkour_debug_enabled.get();
	if (curve.is_valid() && visible) {
		Ref<ImmediateMesh> mesh;
		mesh.instantiate();
		mesh->surface_begin(Mesh::PRIMITIVE_LINE_STRIP);
		for (int i = 0; i < curve->get_point_count(); i++) {
			mesh->surface_add_vertex(curve->get_point_position(i));
		}
		mesh->surface_end();
		MeshInstance3D *mesh_instance = memnew(MeshInstance3D);
		mesh_instance->set_mesh(mesh);
		add_child(mesh_instance);
	}

	if (!visible) {
		Node *node = get_node(NodePath("preview"));
		if (node != nullptr) {
			node->queue_free();
		}
	}
}

void HBAgentParkourLedge::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_curve"), &HBAgentParkourLedge::get_curve);
	ClassDB::bind_method(D_METHOD("set_curve", "curve"), &HBAgentParkourLedge::set_curve);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve3D"), "set_curve", "get_curve");

	ClassDB::bind_method(D_METHOD("get_closed"), &HBAgentParkourLedge::get_closed);
	ClassDB::bind_method(D_METHOD("set_closed", "closed"), &HBAgentParkourLedge::set_closed);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "closed"), "set_closed", "get_closed");
}

Ref<Curve3D> HBAgentParkourLedge::get_curve() const { return curve; }

void HBAgentParkourLedge::set_curve(const Ref<Curve3D> &p_curve) { curve = p_curve; }

void HBAgentParkourLedge::generate_colliders() {
	ERR_FAIL_COND(!curve.is_valid());

	for (int i = 0; i < curve->get_point_count() - 1; i++) {
		Ref<BoxShape3D> box_shape;
		box_shape.instantiate();
		Vector3 a = curve->get_point_position(i);
		Vector3 b = curve->get_point_position((i + 1) % curve->get_point_count());
		Vector3 a_b = b - a;

		Vector3 size = Vector3(0.1, 0.1, 0.0);
		size.z = a_b.length();
		if (Math::is_zero_approx(size.z)) {
			continue;
		}
		box_shape->set_size(size);

		Vector3 pos = a + a_b * 0.5f;
		Quaternion rot = Quaternion(Vector3(0.0, 0.0, -1.0f), a_b.normalized());

		CollisionShape3D *col = memnew(CollisionShape3D);
		col->set_shape(box_shape);
		col->set_quaternion(rot);
		col->set_position(pos);

		add_child(col);

		col->set_owner(get_tree()->get_edited_scene_root());
	}
}

void zfill_callback(const Clipper2Lib::Point64 &e1bot, const Clipper2Lib::Point64 &e1top, const Clipper2Lib::Point64 &e2bot, const Clipper2Lib::Point64 &e2top, Clipper2Lib::Point64 &pt) {
	const double scale = std::pow(10, 2);
	Vector3 a0 = Vector3(e1bot.x, e1bot.z, e1bot.y) / scale;
	Vector3 a1 = Vector3(e1top.x, e1top.z, e1top.y) / scale;
	Vector3 b0 = Vector3(e2bot.x, e2bot.z, e2bot.y) / scale;
	Vector3 b1 = Vector3(e2top.x, e2top.z, e2top.y) / scale;

	Vector3 ps, qt;
	Geometry3D::get_closest_points_between_segments(a0, a1, b0, b1, ps, qt);
	real_t dist = ps.distance_to(qt);
	if (dist < CMP_EPSILON) {
		pt.z = ps.y * scale;
	} else {
		pt.z = (e1bot.z + e2bot.z + e1top.z + e2top.z) / 4;
	}
}

inline Clipper2Lib::PathsD InflatePathsMagic(const Clipper2Lib::PathsD &paths, double delta,
		Clipper2Lib::JoinType jt, Clipper2Lib::EndType et, double miter_limit = 2.0,
		int precision = 2, double arc_tolerance = 0.0) {
	int error_code = 0;
	Clipper2Lib::CheckPrecision(precision, error_code);
	if (!delta)
		return paths;
	if (error_code)
		return Clipper2Lib::PathsD();
	const double scale = std::pow(10, precision);
	Clipper2Lib::ClipperOffset clip_offset(miter_limit, arc_tolerance);
	clip_offset.SetZCallback(zfill_callback);
	clip_offset.PreserveCollinear(true);
	clip_offset.AddPaths(Clipper2Lib::ScalePaths<int64_t, double>(paths, scale, error_code), jt, et);
	if (error_code)
		return Clipper2Lib::PathsD();
	Clipper2Lib::Paths64 solution;
	clip_offset.Execute(delta * scale, solution);
	return Clipper2Lib::ScalePaths<double, int64_t>(solution, 1 / scale, error_code);
}

void HBAgentParkourLedge::generate_offset_path() {
	Clipper2Lib::PathD offset_path;

	for (int j = 0; j < curve->get_point_count(); j++) {
		Vector3 point = curve->get_point_position(j);
		offset_path.push_back(Clipper2Lib::PointD(point.x, point.z, point.y * 1000));
	}

	Clipper2Lib::PathsD paths;
	paths.push_back(offset_path);

	float delta_mul = Clipper2Lib::IsPositive(paths[0]) ? 1.0f : -1.0f;
	Clipper2Lib::PathsD out_paths = InflatePathsMagic(paths, 0.2f * delta_mul, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon, 2.0, 2);
}

void HBAgentParkourLedge::round_path() {
	// Rounds the path, must NOT be closed yet
	const float AGENT_RADIUS = 0.2f;

	struct Point {
		bool is_curved = false;
		Vector3 in;
		Vector3 out;
		Vector3 pos;
	} points[256] = {};

	int point_count = 0;

	for (int i = 0; i < curve->get_point_count(); i++) {
		int prev_point_idx = Math::posmod(i - 1, curve->get_point_count());
		int next_point_idx = Math::posmod(i + 1, curve->get_point_count());
		Vector3 point_prev = curve->get_point_position(prev_point_idx);
		if (point_count > 0) {
			point_prev = points[point_count - 1].pos;
		}
		Vector3 point = curve->get_point_position(i);
		Vector3 point_next = curve->get_point_position(next_point_idx);

		const Vector3 dir_to_prev = point.direction_to(point_prev);
		const Vector3 dir_to_next = point.direction_to(point_next);

		bool is_end = i == 0 || (i == curve->get_point_count() - 1);

		if (dir_to_prev.angle_to(dir_to_next) < Math::deg_to_rad(35.0f) || (!closed && is_end)) {
			points[point_count++].pos = point;
			continue;
		}

		const float dist_prev = point.distance_to(point_prev);
		const float dist_next = point.distance_to(point_next);

		const float interp_radius_prev = MIN(dist_prev, AGENT_RADIUS);
		const float interp_radius_next = MIN(dist_next, AGENT_RADIUS);

		const Vector3 p0 = point + dir_to_prev * interp_radius_prev;
		const Vector3 p2 = point + dir_to_next * interp_radius_next;

		const Vector3 p0x = p0.lerp(point, 0.5f);
		const Vector3 p2x = p2.lerp(point, 0.5f);

		const Vector3 curve_mid_point = p0x.lerp(p2x, 0.5f);

		if (dist_prev > AGENT_RADIUS) {
			// No need to add the previous point if it's too close to us
			const int prev_idx = point_count++;
			points[prev_idx].pos = p0;
		}
		const int curr_idx = point_count++;
		const int next_idx = point_count++;

		points[curr_idx].is_curved = true;
		points[curr_idx].in = p0x - curve_mid_point;
		points[curr_idx].out = p2x - curve_mid_point;
		points[curr_idx].pos = curve_mid_point;
		points[next_idx].pos = p2;

		if (dist_next <= AGENT_RADIUS) {
			// Skip next point if our out control point was too close.
			i++;
		}
	}

	Ref<Curve3D> rounded_path;
	rounded_path.instantiate();

	for (int i = 0; i < point_count; i++) {
		rounded_path->add_point(points[i].pos, points[i].in, points[i].out);
	}

	curve = rounded_path;
}

float HBAgentParkourLedge::get_closest_offset(const Vector3 &p_global_pos) const {
	ERR_FAIL_COND_V(!curve.is_valid(), -1.0f);
	return curve->get_closest_offset(to_local(p_global_pos));
}

Transform3D HBAgentParkourLedge::get_ledge_transform_at_offset(float p_offset) const {
	ERR_FAIL_COND_V(!curve.is_valid(), Transform3D());
	Transform3D trf = curve->sample_baked_with_rotation(Math::fposmod(p_offset, curve->get_baked_length()));
	Vector3 forward = trf.basis.get_column(2);
	trf.basis = Basis::looking_at(-forward, Vector3(0.0f, 1.0f, 0.0f));
	Quaternion rot = Quaternion(Vector3(0.0, 0.0, -1.0f), trf.basis.xform(Vector3(1.0f, 0.0f, 0.0f)));
	rot = Quaternion(rot.xform(Vector3(0.0f, 1.0f, 0.0f)), Vector3(0.0f, 1.0f, 0.0f)) * rot;
	trf.basis = rot;
	return trf;
}

Transform3D HBAgentParkourLedge::get_agent_ledge_transform_at_offset(float p_offset) const {
	ERR_FAIL_COND_V(!curve.is_valid(), Transform3D());
	Transform3D curve_trf = get_ledge_transform_at_offset(p_offset);
	const float AGENT_RADIUS = 0.2f;
	curve_trf.origin = curve_trf.xform(Vector3(0.0, 0.0, AGENT_RADIUS));
	return curve_trf;
}

bool HBAgentParkourLedge::check_agent_fits(HBAgent *p_agent, float p_offset, HBDebugGeometry *p_debug_geo) const {
	PhysicsDirectSpaceState3D *dss = p_agent->get_world_3d()->get_direct_space_state();
	PhysicsDirectSpaceState3D::ShapeParameters shape_params;
	shape_params.collision_mask = HBPhysicsLayers::LAYER_WORLD_GEO;
	Transform3D ledge_trf = get_ledge_transform_at_offset(p_offset);
	Ref<Shape3D> shape = p_agent->get_collision_shape();
	shape_params.shape_rid = shape->get_rid();
	shape_params.transform.origin = ledge_trf.origin + ledge_trf.basis.xform(Vector3(0.0, 0.0, p_agent->get_radius() + 0.1f));
	PhysicsDirectSpaceState3D::ShapeRestInfo rest_info;
	bool hit = dss->rest_info(shape_params, &rest_info);
	if (p_debug_geo) {
		p_debug_geo->debug_cast_motion(shape, shape_params, Color("BLUE"));
	}
	return !hit;
}

HBAgentParkourLedge::HBAgentParkourLedge() {
	set_collision_mask(0);
	set_collision_layer(HBPhysicsLayers::LAYER_PARKOUR_NODES);
	parkour_debug_enabled.data->get_signaler()->connect("changed", callable_mp(this, &HBAgentParkourLedge::_on_parkour_debug_changed));
}

void HBAgentParkourLedge::_editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) {
	if (p_info.parents.size() > 0) {
		queue_free();
		return;
	}
	Vector<StringName> children = p_info.children;
	HBAgentParkourLedge *ledge = Object::cast_to<HBAgentParkourLedge>(p_info.node);

	DEV_ASSERT(ledge != nullptr);

	curve.instantiate();
	curve->add_point(get_global_position());

	while (children.size() > 0) {
		StringName child = children[0];
		const EntityCompileInfo *child_info = p_entities.getptr(child);
		if (!child_info) {
			break;
		}

		curve->add_point(child_info->node->get_global_position());

		children = child_info->children;
	}

	round_path();

	if (ledge->closed) {
		curve->add_point(curve->get_point_position(0));
	}

	set_global_position(Vector3());
	set_global_basis(Quaternion());

	generate_colliders();
	generate_offset_path();
}
