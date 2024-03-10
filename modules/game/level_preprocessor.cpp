/**************************************************************************/
/*  level_preprocessor.cpp                                                */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
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

#include "level_preprocessor.h"
#include "modules/csg/csg.h"
#include "modules/tbloader/src/map/surface_gatherer.h"
#include "scene/resources/3d/concave_polygon_shape_3d.h"
#include "scene/resources/mesh_data_tool.h"
#include "thirdparty/clipper_3d/clipper_3d.h"
#include <iterator>

struct Group {
	Vector<Vector3> triangles;
};

static bool flag = false;

bool try_merge_groups(Vector<Vector3> p_a, Vector<Vector3> p_b) {
	for (int i = 0; i < p_a.size(); i++) {
		Vector3 e0v0 = p_a[i];
		Vector3 e0v1 = p_a[(i + 1) % p_a.size()];
		for (int j = 0; j < p_b.size(); j++) {
			Vector3 e1v0 = p_b[j];
			Vector3 e1v1 = p_b[(j + 1) % p_b.size()];
			real_t dist = Geometry3D::get_closest_distance_between_segments(e0v0, e0v1, e1v0, e1v1);
			real_t angle = (e0v1 - e0v0).angle_to(e1v1 - e1v0);
			if (dist < 0.001 && (Math::is_zero_approx(angle) || Math::is_equal_approx(angle, Math::deg_to_rad(180.0f)))) {
				if (flag) {
					print_line("UOHHH", e0v0, e0v1, e1v0, e1v1, Math::rad_to_deg(angle), dist);
				}
				return true;
			}
		}
	}

	return false;
}

bool try_merge_buckets(const HBLevelPreprocessor::CollisionBucket &p_a, const HBLevelPreprocessor::CollisionBucket &p_b) {
	for (int i = 0; i < p_a.polylines.size(); i++) {
		for (int k = 0; k < p_b.polylines.size(); k++) {
			if (try_merge_groups(p_a.polylines[i], p_b.polylines[k])) {
				return true;
			}
		}
	}
	return false;
}

#define SCALE_FACTOR 100000.0 // Based on CMP_EPSILON.

void zfill_callback(ClipperLib3D::IntPoint &e1bot, ClipperLib3D::IntPoint &e1top, ClipperLib3D::IntPoint &e2bot, ClipperLib3D::IntPoint &e2top, ClipperLib3D::IntPoint &pt) {
	Vector3 a0 = Vector3(e1bot.X, e1bot.Z, e1bot.Y) / SCALE_FACTOR;
	Vector3 a1 = Vector3(e1top.X, e1top.Z, e1top.Y) / SCALE_FACTOR;
	Vector3 b0 = Vector3(e2bot.X, e2bot.Z, e2bot.Y) / SCALE_FACTOR;
	Vector3 b1 = Vector3(e2top.X, e2top.Z, e2top.Y) / SCALE_FACTOR;

	Vector3 ps, qt;
	Geometry3D::get_closest_points_between_segments(a0, a1, b0, b1, ps, qt);
	real_t dist = ps.distance_to(qt);
	if (dist < CMP_EPSILON) {
		pt.Z = ps.y * SCALE_FACTOR;
	} else {
		pt.Z = (e1bot.Z + e2bot.Z + e1top.Z + e2top.Z) / 4;
	}
}

Vector<Vector<Vector3>> funny_union(const Vector<Vector3> &p_polypath_a, const Vector<Vector3> &p_polypath_b, bool is_a_open = false, bool p_preserve_colinear = true) {
	using namespace ClipperLib3D;

	ClipType op = ctUnion;
	Path path_a;

	// Need to scale points (Clipper's requirement for robust computation).
	for (int i = 0; i != p_polypath_a.size(); ++i) {
		path_a << IntPoint(p_polypath_a[i].x * (real_t)SCALE_FACTOR, p_polypath_a[i].z * (real_t)SCALE_FACTOR, p_polypath_a[i].y * (real_t)SCALE_FACTOR);
	}
	Clipper clp;
	clp.PreserveCollinear(p_preserve_colinear);
	clp.ZFillFunction(zfill_callback);
	clp.AddPath(path_a, ptSubject, !is_a_open); // Forward compatible with Clipper 10.0.0.

	Path path_b;
	for (int i = 0; i != p_polypath_b.size(); ++i) {
		path_b << IntPoint(p_polypath_b[i].x * (real_t)SCALE_FACTOR, p_polypath_b[i].z * (real_t)SCALE_FACTOR, p_polypath_b[i].y * (real_t)SCALE_FACTOR);
	}
	clp.AddPath(path_b, ptClip, true); // Polylines cannot be set as clip.

	Paths paths;

	if (is_a_open) {
		PolyTree tree; // Needed to populate polylines.
		clp.Execute(op, tree);
		OpenPathsFromPolyTree(tree, paths);
	} else {
		clp.Execute(op, paths); // Works on closed polygons only.
	}
	// Have to scale points down now.
	Vector<Vector<Vector3>> polypaths;

	for (Paths::size_type i = 0; i < paths.size(); ++i) {
		Vector<Vector3> polypath;

		const Path &scaled_path = paths[i];

		for (Paths::size_type j = 0; j < scaled_path.size(); ++j) {
			polypath.push_back(Vector3(
					static_cast<real_t>(scaled_path[j].X) / (real_t)SCALE_FACTOR,
					static_cast<real_t>(scaled_path[j].Z) / (real_t)SCALE_FACTOR,
					static_cast<real_t>(scaled_path[j].Y) / (real_t)SCALE_FACTOR));
		}
		polypaths.push_back(polypath);
	}
	return polypaths;
}

TypedArray<PackedVector3Array> process_mesh(Vector<Vector3> p_faces) {
	LocalVector<Group> groups;

	Vector<HBLevelPreprocessor::CollisionBucket> collision_buckets = HBLevelPreprocessor::bucketify(p_faces);

	TypedArray<PackedVector3Array> polygons;
	for (int i = 0; i < collision_buckets.size(); i++) {
		polygons.push_back(collision_buckets[i].polyline);
	}

	return polygons;
}

void HBLevelPreprocessor::_bind_methods() {
}

PackedVector3Array HBLevelPreprocessor::filter_ledge_geometry(Ref<ConcavePolygonShape3D> p_world_geometry) {
	PackedVector3Array geo = p_world_geometry->get_faces();
	// Put all faces that are touching through common vertices and that are facing up
	PackedVector3Array out_geo;
	for (int i = 0; i < geo.size(); i += 3) {
		Vector3 e1 = geo[i + 1] - geo[i];
		Vector3 e2 = geo[i + 2] - geo[i + 1];
		if (e2.cross(e1).angle_to(Vector3(0.0, 1.0, 0.0)) > Math::deg_to_rad(45.0f)) {
			continue;
		}
		out_geo.push_back(geo[i]);
		out_geo.push_back(geo[i + 1]);
		out_geo.push_back(geo[i + 2]);
	}
	return out_geo;
}

Vector<HBLevelPreprocessor::CollisionBucket> HBLevelPreprocessor::bucketify(Vector<Vector3> p_faces) {
	Vector<CollisionBucket> buckets;

	for (int i = 0; i < p_faces.size(); i += 3) {
		CollisionBucket bucket;
		PackedVector3Array pl;
		pl.push_back(p_faces[i]);
		pl.push_back(p_faces[i + 1]);
		pl.push_back(p_faces[i + 2]);
		pl.push_back(p_faces[i]);
		bucket.polylines.push_back(pl);
		buckets.push_back(bucket);
	}
	bool changed = true;
	while (changed) {
		changed = false;

		for (int i = buckets.size() - 1; i >= 0; i--) {
			for (int j = i - 1; j >= 0; j--) {
				bool result = try_merge_buckets(buckets[j], buckets[i]);
				if (result) {
					changed = true;
					Vector<Vector<Vector3>> united = funny_union(buckets[j].polyline, buckets[i].polyline, false, false);
					buckets.ptrw()[j].polylines.append_array(buckets[i].polylines);
					buckets.remove_at(i);
					break;
				}
			}
			if (changed) {
				break;
			}
		}
	}

	Vector<CollisionBucket> real_buckets;

	for (int i = 0; i < buckets.size(); i++) {
		Vector<Vector<Vector3>> polys_to_merge = buckets[i].polylines;
		Vector<Vector<Vector3>> out;
		out.push_back(polys_to_merge[polys_to_merge.size() - 1]);
		polys_to_merge.remove_at(polys_to_merge.size() - 1);
		changed = true;
		while (changed && polys_to_merge.size() > 0) {
			changed = false;
			for (int k = polys_to_merge.size() - 1; k >= 0; k--) {
				for (int l = out.size() - 1; l >= 0; l--) {
					bool res = try_merge_groups(polys_to_merge[k], out[l]);
					if (res) {
						Vector<Vector<Vector3>> union_out = funny_union(polys_to_merge[k], out[l], false, false);
						out.ptrw()[l] = union_out[0];
						if (union_out.size() > 0) {
							for (int z = 1; z < union_out.size(); z++) {
								out.push_back(union_out[z]);
							}
						}
						polys_to_merge.remove_at(k);
						changed = true;
						break;
					}
				}
			}
		}
		changed = true;
		int max_iters = 1000;
		int iters = 0;
		while (changed && iters < max_iters) {
			changed = false;
			for (int j = 0; j < out.size(); j++) {
				for (int l = 0; l < j; l++) {
					if (j == l) {
						continue;
					}
					bool res = try_merge_groups(out[j], out[l]);
					if (res) {
						Vector<Vector<Vector3>> union_out1 = funny_union(out[j], out[l], false, false);
						Vector<Vector<Vector3>> union_out2 = funny_union(out[j], out[l], false, false);
						Vector<Vector<Vector3>> outt = union_out1;
						if (union_out2.size() > union_out1.size()) {
							outt = union_out2;
						}
						if (outt.size() > 0) {
							out.remove_at(j);
							out.remove_at(l);
							for (int z = 0; z < outt.size(); z++) {
								out.push_back(outt[z]);
							}
							changed = true;
							break;
						}
					}
				}
				if (changed) {
					break;
				}
			}
			iters++;
		}

		for (int j = 0; j < out.size(); j++) {
			CollisionBucket bucket;
			bucket.polyline = out[j];
			real_buckets.push_back(bucket);
		}
	}
	return real_buckets;
}

Vector<Vector<Vector3>> HBLevelPreprocessor::process(Vector<Vector3> p_faces) {
	LocalVector<Group> groups;

	Vector<HBLevelPreprocessor::CollisionBucket> collision_buckets = HBLevelPreprocessor::bucketify(p_faces);

	Vector<Vector<Vector3>> polygons;
	for (int i = 0; i < collision_buckets.size(); i++) {
		polygons.push_back(collision_buckets[i].polyline);
	}

	return polygons;
}

struct CSGBucket {
	Vector<Vector3> vertices;
	bool try_merge(const CSGBucket &p_b) const {
		const CSGBucket &p_a = *this;

		for (int i_a = 0; i_a < p_a.vertices.size(); i_a += 3) {
			for (int tri_a = 0; tri_a < 3; tri_a++) {
				Vector3 e0v0 = p_a.vertices[i_a + tri_a];
				Vector3 e0v1 = p_a.vertices[i_a + ((tri_a + 1) % 3)];

				for (int i_b = 0; i_b < p_b.vertices.size(); i_b += 3) {
					for (int tri_b = 0; tri_b < 3; tri_b++) {
						Vector3 e1v0 = p_b.vertices[i_b + tri_b];
						Vector3 e1v1 = p_b.vertices[i_b + ((tri_b + 1) % 3)];
						real_t dist = Geometry3D::get_closest_distance_between_segments(e0v0, e0v1, e1v0, e1v1);
						real_t angle = (e0v1 - e0v0).angle_to(e1v1 - e1v0);
						if (dist < 0.001 && (Math::is_zero_approx(angle) || Math::is_equal_approx(angle, Math::deg_to_rad(180.0f)))) {
							return true;
						}
					}
				}
			}
		}

		return false;
	}
};
