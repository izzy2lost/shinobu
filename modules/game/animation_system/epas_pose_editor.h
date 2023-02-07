#ifndef EPAS_POSE_EDITOR_H
#define EPAS_POSE_EDITOR_H
#ifdef DEBUG_ENABLED
#include "core/object/undo_redo.h"
#include "epas_controller.h"
#include "epas_editor_camera.h"
#include "epas_editor_grid.h"
#include "epas_pose.h"
#include "epas_pose_node.h"
#include "modules/imgui/godot_imgui.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/gui/file_dialog.h"

class EPASPoseEditor : public Control {
	GDCLASS(EPASPoseEditor, Control);

private:
	PackedFloat32Array view_matrix;
	PackedFloat32Array projection_matrix;
	PackedFloat32Array selected_bone_trf_matrix;
	PackedFloat32Array editing_bone_trf_matrix;
	PackedFloat32Array identity_matrix;
	bool was_using_gizmo = false;

	ImGuizmo::OPERATION guizmo_operation = ImGuizmo::ROTATE;
	ImGuizmo::MODE guizmo_mode = ImGuizmo::LOCAL;

	FileDialog *file_open_dialog;
	FileDialog *model_load_dialog;
	FileDialog *save_file_dialog;

	EPASEditorCamera *camera;
	Node3D *editor_3d_root;
	EPASEditorGrid *grid;

	Node3D *editing_model = nullptr;
	Skeleton3D *editing_skeleton = nullptr;
	Ref<EPASPose> current_pose;
	int editing_bone = -1;

	EPASController *epas_controller;
	Ref<EPASPoseNode> epas_pose_node;
	Ref<Font> label_font;
	int currently_hovered_bone = -1;

	bool control = false;

	UndoRedo *undo_redo = nullptr;

	void _draw_ui();
	void _draw_bone_positions();
	void _update_editing_skeleton_trf();
	void _world_to_bone_trf(int p_bone_idx, const float *p_world_trf, Transform3D &p_out);

protected:
	void _notification(int p_what);
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;

public:
	Transform3D get_edited_object_transform();
	void open_file(const String &p_path);
	void load_model(const String &p_path);
	void set_pose(const Ref<EPASPose> &p_pose);
	void save_to_path(const String &p_path);

	EPASPoseEditor();
	virtual ~EPASPoseEditor();
};
#endif // DEBUG_ENABLED
#endif // EPAS_POSE_EDITOR_H