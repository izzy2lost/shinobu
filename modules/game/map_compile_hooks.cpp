#include "map_compile_hooks.h"
#include "modules/game/agent_parkour.h"
#include "modules/game/level_preprocessor.h"
#include "scene/resources/concave_polygon_shape_3d.h"

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
	}
}
