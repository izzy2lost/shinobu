#include "map_compile_hooks.h"
#include "modules/game/agent_parkour.h"
#include "modules/game/level_preprocessor.h"
#include "scene/resources/concave_polygon_shape_3d.h"
#include "clipper2/clipper.h"


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

inline Clipper2Lib::PathsD InflatePathsMagic(const Clipper2Lib::PathsD& paths, double delta,
	Clipper2Lib::JoinType jt, Clipper2Lib::EndType et, double miter_limit = 2.0, 
	int precision = 2, double arc_tolerance = 0.0)
{
	int error_code = 0;
	Clipper2Lib::CheckPrecision(precision, error_code);
	if (!delta) return paths;
	if (error_code) return Clipper2Lib::PathsD();
	const double scale = std::pow(10, precision);
	Clipper2Lib::ClipperOffset clip_offset(miter_limit, arc_tolerance);
	clip_offset.SetZCallback(zfill_callback);
	clip_offset.PreserveCollinear(true);
	clip_offset.AddPaths(Clipper2Lib::ScalePaths<int64_t,double>(paths, scale, error_code), jt, et);
	if (error_code) return Clipper2Lib::PathsD();
	Clipper2Lib::Paths64 solution;
	clip_offset.Execute(delta * scale, solution);
	return Clipper2Lib::ScalePaths<double, int64_t>(solution, 1 / scale, error_code);
}

void MapCompileHooks::post_compile_hook(TBLoader *p_loader, TBLoaderBuildInfo *p_build_info) {
	Vector<Vector3> faces;
	Ref<HBLevelPreprocessor> preprop;
	preprop.instantiate();
	for (TBLoaderEntityBuildInfo &info : p_build_info->entities) {
		if (!info.collision_shape) {
			continue;
		}
		if (info.class_name != "worldspawn") {
			continue;
		}
		Ref<ConcavePolygonShape3D> shape = info.collision_shape->get_shape();
		if (!shape.is_valid()) {
			continue;
		}

		Vector<Vector3> poly_faces = HBLevelPreprocessor::filter_ledge_geometry(shape);

		for (Vector3 &point : poly_faces) {
			point = info.collision_shape->get_global_transform().xform(point);
		}

		faces.append_array(poly_faces);
	}

	Vector<Vector<Vector3>> polys = HBLevelPreprocessor::process(faces);

	Node3D *path_container = memnew(Node3D);
	path_container->set_name("Ledges");
	p_loader->add_child(path_container);
	path_container->set_owner(p_loader->get_tree()->get_edited_scene_root());

	for (int i = 0; i < polys.size(); i++) {
		Ref<Curve3D> curve;
		curve.instantiate();

		HBAgentParkourLedge *ledge = memnew(HBAgentParkourLedge);
		ledge->set_curve(curve);

		for (int j = 0; j < polys[i].size(); j++) {
			curve->add_point(polys[i][j]);
		}
		curve->add_point(polys[i][0]);

		Path3D *path = memnew(Path3D);
		path->set_curve(curve);
		path_container->add_child(path);
		path->set_owner(p_loader->get_tree()->get_edited_scene_root());
		path->set_name(vformat("%d", i));

		path_container->add_child(ledge);
		ledge->set_owner(p_loader->get_tree()->get_edited_scene_root());
		ledge->generate_colliders();

		Clipper2Lib::PathD offset_path;

		for (int j = 0; j < polys[i].size(); j++) {
			offset_path.push_back(Clipper2Lib::PointD(polys[i][j].x, polys[i][j].z, polys[i][j].y * 1000));
		}

		Clipper2Lib::PathsD paths;
		paths.push_back(offset_path);

		float delta_mul = Clipper2Lib::IsPositive(paths[0]) ? 1.0f : -1.0f;
		Clipper2Lib::PathsD out_paths = InflatePathsMagic(paths, 0.2f * delta_mul, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon, 2.0, 2);
		
		if (out_paths.size() > 0) {
			Path3D *offset_path_preview = memnew(Path3D);
			Ref<Curve3D> offset_curve;
			offset_curve.instantiate();
			for (const Clipper2Lib::PointD &point : out_paths[0]) {
				offset_curve->add_point(Vector3(point.x, point.z / 1000.0f, point.y));
			}
			ledge->set_agent_curve(offset_curve);
			offset_curve->add_point(offset_curve->get_point_position(0));

			offset_path_preview->set_curve(offset_curve);
			ledge->add_child(offset_path_preview);
			offset_path_preview->set_name(delta_mul == 1.0 ? "POSITIVE" : "NEGATIVE");
			offset_path_preview->set_owner(p_loader->get_tree()->get_edited_scene_root());
		}
	}
}
