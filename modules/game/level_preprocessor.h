#ifndef LEVEL_PREPROCESSOR_H
#define LEVEL_PREPROCESSOR_H

#include "core/math/geometry_2d.h"
#include "core/math/triangulate.h"
#include "core/math/vector3.h"
#include "core/templates/local_vector.h"
#include "modules/tbloader/src/map/map_data.h"
#include "scene/resources/mesh.h"

class HBLevelPreprocessor : public RefCounted {
	GDCLASS(HBLevelPreprocessor, RefCounted);

protected:
	static void _bind_methods();

public:
	struct CollisionBucket {
		Vector<Vector<Vector3>> polylines;
		Vector<Vector3> polyline;
	};
	static PackedVector3Array filter_ledge_geometry(Ref<ConcavePolygonShape3D> p_world_geometry);
	static Vector<HBLevelPreprocessor::CollisionBucket> bucketify(Vector<Vector3> p_faces);
	static Vector<Vector<Vector3>> process(Vector<Vector3> p_faces);
};

#endif // LEVEL_PREPROCESSOR_H
