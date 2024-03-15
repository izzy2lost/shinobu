/**************************************************************************/
/*  level_preprocessor.h                                                  */
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
