/**************************************************************************/
/*  epas_editor_grid.cpp                                                  */
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

#include "epas_editor_grid.h"
#include "scene/resources/immediate_mesh.h"

void EPASEditorGrid::set_grid_size(float p_grid_size) {
	grid_size = p_grid_size;
	grid_dirty = true;
	_update_grid();
}

float EPASEditorGrid::get_grid_size() const {
	return grid_size;
}

void EPASEditorGrid::_update_grid() {
	if (!grid_dirty) {
		return;
	}
	grid_dirty = false;

	Ref<ImmediateMesh> im_mesh = mesh_instance->get_mesh();
	ERR_FAIL_COND(im_mesh.is_null());
	im_mesh->clear_surfaces();
	im_mesh->surface_begin(Mesh::PrimitiveType::PRIMITIVE_LINES);

	float max = grid_size * (GRID_LINES / float(2.0));
	float min = -max;

	for (int i = 0; i < GRID_LINES; i++) {
		float dist = Math::lerp(min, max, i / float(GRID_LINES - 1));
		im_mesh->surface_add_vertex(Vector3(min, 0.0, dist));
		im_mesh->surface_add_vertex(Vector3(max, 0.0, dist));
		im_mesh->surface_add_vertex(Vector3(dist, 0.0, min));
		im_mesh->surface_add_vertex(Vector3(dist, 0.0, max));
	}
	im_mesh->surface_end();
	im_mesh->surface_set_material(0, mat);
}

void EPASEditorGrid::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			_update_grid();
		} break;
	}
}

EPASEditorGrid::EPASEditorGrid() {
	mesh_instance = memnew(MeshInstance3D);
	add_child(mesh_instance);
	Ref<ImmediateMesh> im_mesh = memnew(ImmediateMesh);
	mesh_instance->set_mesh(im_mesh);
	mat = Ref<StandardMaterial3D>(memnew(StandardMaterial3D));
	mat->set_shading_mode(BaseMaterial3D::ShadingMode::SHADING_MODE_UNSHADED);
	mat->set_albedo(Color(0.5, 0.5, 0.5, 1.0));
}