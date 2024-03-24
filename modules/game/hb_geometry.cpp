/**************************************************************************/
/*  hb_geometry.cpp                                                       */
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

#include "hb_geometry.h"
#include "core/variant/typed_array.h"
#include "thirdparty/stb/stb_rect_pack.h"

void HBGeometry::_bind_methods() {
	ClassDB::bind_static_method("HBGeometry", D_METHOD("pack_rects", "dimensions", "rects"), &HBGeometry::pack_rects);
}

Dictionary HBGeometry::pack_rects(Vector2i p_dimensions, TypedArray<Vector2i> p_rects) {
	stbrp_context context;
	Vector<stbrp_node> nodes;
	nodes.resize(p_dimensions.x);
	stbrp_node *nodes_ptrw = nodes.ptrw();

	stbrp_init_target(&context, p_dimensions.x, p_dimensions.y, nodes_ptrw, nodes.size());

	Vector<stbrp_rect> rects;
	rects.resize(p_rects.size());
	stbrp_rect *rects_ptrw = rects.ptrw();

	for (int i = 0; i < p_rects.size(); i++) {
		Vector2i rect = p_rects[i];
		rects_ptrw[i].id = i;
		rects_ptrw[i].w = rect.x;
		rects_ptrw[i].h = rect.y;
	}

	int result = stbrp_pack_rects(&context, rects_ptrw, rects.size());
	ERR_FAIL_COND_V_MSG(result != 1, Dictionary(), "Error packing rectangles");

	Dictionary out_dict;

	TypedArray<Vector2i> out_positions;
	out_positions.resize(rects.size());
	for (int i = 0; i < rects.size(); i++) {
		out_positions[i] = Vector2i(rects[i].x, rects[i].y);
	}
	out_dict["points"] = out_positions;
	return out_dict;
}
