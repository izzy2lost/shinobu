#ifndef EPAS_ANIMATION_EDITOR_H
#define EPAS_ANIMATION_EDITOR_H
#include "core/error/error_macros.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_animation_node.h"
#ifdef DEBUG_ENABLED
#include "core/object/ref_counted.h"
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

class EPASAnimationEditor : public Control {
	GDCLASS(EPASAnimationEditor, Control);

private:
	static const int FPS = 60; // harcoded to 60... for now
	// When manipulating keyframes we need to make sure the order of keyframes doesn't change
	// however, keyframes are reordered when their time is changed, so we have to keep track of
	// them separatedly so their indices don't change
	struct AnimationKeyframeCache {
		Ref<EPASKeyframe> keyframe;
		int32_t frame_time = 0;
		int32_t temporary_frame_time = 0;
		AnimationKeyframeCache(Ref<EPASKeyframe> p_keyframe) {
			keyframe = p_keyframe;
			frame_time = keyframe->get_time() * FPS;
			temporary_frame_time = frame_time;
		}
		void apply_temp_frame_time() {
			keyframe->set_time(temporary_frame_time / (float)FPS);
		}
		void apply_frame_time() {
			keyframe->set_time(frame_time / (float)FPS);
		}
	};
	Vector<AnimationKeyframeCache *> keyframe_cache;

	uint32_t current_frame = 0;
	uint32_t start_frame = 0;
	uint32_t end_frame = 100;

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

	Node3D *current_model = nullptr;
	Skeleton3D *editing_skeleton = nullptr;
	Ref<EPASAnimation> current_animation;

	int editing_bone = -1;

	EPASController *epas_controller;
	Ref<EPASAnimationNode> epas_animation_node;
	Ref<Font> label_font;
	int currently_hovered_bone = -1;

	bool control = false;

	UndoRedo *undo_redo = nullptr;

	void _rebuild_keyframe_cache();
	void _draw_ui();
	void _draw_bone_positions();
	void _update_editing_skeleton_trf();
	void _world_to_bone_trf(int p_bone_idx, const float *p_world_trf, Transform3D &p_out);
	void _set_frame_time(int p_frame_idx, int32_t p_frame_time);

	Ref<EPASPose> get_current_pose() const;

protected:
	void _notification(int p_what);
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;

public:
	Transform3D get_edited_object_transform();
	void open_file(const String &p_path);
	void load_model(const String &p_path);
	void set_animation(const Ref<EPASAnimation> &p_animation);
	Ref<EPASAnimation> get_animation() const;
	void save_to_path(const String &p_path);

	EPASAnimationEditor();
	virtual ~EPASAnimationEditor();
};
#endif // DEBUG_ENABLED
#endif // EPAS_ANIMATION_EDITOR_H