#ifndef EPAS_EDITOR_GRID_H
#define EPAS_EDITOR_GRID_H

#include "scene/3d/mesh_instance_3d.h"

class EPASEditorGrid : public Node3D {
	GDCLASS(EPASEditorGrid, Node3D);
	const int GRID_LINES = 10;
	float grid_size = 0.5f;
	bool grid_dirty = true;
	Ref<StandardMaterial3D> mat;
	MeshInstance3D *mesh_instance;
	void _update_grid();

protected:
	void _notification(int p_what);

public:
	void set_grid_size(float p_grid_size);
	float get_grid_size() const;
	EPASEditorGrid();
};

#endif // EPAS_EDITOR_GRID_H