#ifndef EPAS_ANIMATION_EDITOR_H
#define EPAS_ANIMATION_EDITOR_H
#include "core/math/transform_3d.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_animation_node.h"
#ifdef DEBUG_ENABLED
#include "../utils.h"
#include "core/object/ref_counted.h"
#include "core/object/undo_redo.h"
#include "epas_controller.h"
#include "epas_editor_camera.h"
#include "epas_editor_grid.h"
#include "epas_pose.h"
#include "modules/imgui/godot_imgui.h"
#include "scene/2d/multimesh_instance_2d.h"
#include "scene/3d/multimesh_instance_3d.h"
#include "scene/3d/node_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/gui/file_dialog.h"

class EPASAnimationEditorIKJoint : public Resource {
	GDCLASS(EPASAnimationEditorIKJoint, Resource);
	String ik_tip_bone_name;
	bool use_magnet;
	// We use a virtual bone in the skeleton for simplifying IK code
	String ik_target_bone_name;
	String ik_magnet_bone_name;

public:
	bool get_use_magnet() const { return use_magnet; }
	void set_use_magnet(bool p_use_magnet) { use_magnet = p_use_magnet; }

	String get_ik_tip_bone_name() const { return ik_tip_bone_name; }
	void set_ik_tip_bone_name(const String &p_ik_tip_bone_name) { ik_tip_bone_name = p_ik_tip_bone_name; }

	String get_ik_target_bone_name() const { return ik_target_bone_name; }
	void set_ik_target_bone_name(const String &p_ik_target_bone_name) { ik_target_bone_name = p_ik_target_bone_name; }

	String get_ik_magnet_bone_name() const { return ik_magnet_bone_name; }
	void set_ik_magnet_bone_name(const String &p_ik_magnet_bone_name) { ik_magnet_bone_name = p_ik_magnet_bone_name; }
};

class EPASAnimationEditor : public Control {
	GDCLASS(EPASAnimationEditor, Control);

private:
	class Selection : public RefCounted {
	public:
		enum SelectionType {
			FK_BONE,
			IK_HANDLE,
			WARP_POINT
		};

		enum SelectionGroup {
			FK_CENTER_GROUP,
			FK_LEFT_GROUP,
			FK_RIGHT_GROUP,
			FK_FINGER_GROUP,
			IK_GROUP,
			WARP_POINT_GROUP,
			GROUP_MAX
		};

		SelectionType type = SelectionType::FK_BONE;
		SelectionGroup group = SelectionGroup::FK_CENTER_GROUP;
		int bone_idx;
		bool hidden = false;
		bool is_ik_magnet = false;
		Ref<EPASAnimationEditorIKJoint> ik_joint;
		Ref<EPASWarpPoint> warp_point;
	};

	static const int FPS = 60; // harcoded to 60... for now
	// When manipulating keyframes we need to make sure the order of keyframes doesn't change
	// however, keyframes are reordered in the animation when their time is changed, so we have
	// to keep track of them separatedly so their indices don't change
	struct AnimationKeyframeCache {
		Ref<EPASKeyframe> keyframe;
		int32_t frame_time = 0;
		int32_t temporary_frame_time = 0;
		AnimationKeyframeCache(Ref<EPASKeyframe> p_keyframe) {
			keyframe = p_keyframe;
			frame_time = Math::round(keyframe->get_time() * FPS);
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
	ImGui::FrameIndexType current_frame = 0;
	ImGui::FrameIndexType start_frame = 0;
	ImGui::FrameIndexType end_frame = 100;

	PackedFloat32Array view_matrix;
	PackedFloat32Array projection_matrix;
	// Old trf matrix, for undo_redo
	PackedFloat32Array prev_handle_trf_matrix;
	// Actual TRF matrix the user is manipulating
	PackedFloat32Array current_handle_trf_matrix;
	PackedFloat32Array identity_matrix;
	bool was_using_gizmo = false;

	ImGuizmo::OPERATION guizmo_operation = ImGuizmo::ROTATE;
	ImGuizmo::MODE guizmo_mode = ImGuizmo::LOCAL;

	FileDialog *file_open_dialog;
	FileDialog *model_load_dialog;
	FileDialog *placeholder_load_dialog;
	FileDialog *save_file_dialog;

	EPASEditorCamera *camera;
	Node3D *editor_3d_root;
	EPASEditorGrid *grid;
	MeshInstance3D *solid_ground;

	Node3D *current_model = nullptr;
	Skeleton3D *editing_skeleton = nullptr;
	Vector<Ref<EPASAnimationEditorIKJoint>> ik_constraints;
	Ref<EPASAnimation> current_animation;
	Vector<Ref<Selection>> selection_handles;
	MultiMeshInstance2D *selection_handle_dots;
	// Spheres that show the location of warp points
	MultiMeshInstance3D *warp_point_spheres;
	Node3D *placeholder_scene;

	Vector<ImGui::FrameIndexType> kf_selection;

	int editing_selection_handle = -1;

	EPASController *epas_controller;
	Ref<EPASAnimationNode> epas_animation_node;
	Ref<Font> label_font;
	int currently_hovered_selection_handle = -1;

	struct ui_info_t {
		bool posedbg_window_visible = false;
		bool constraintdbg_window_visible = false;
		int selected_warp_point = -1;
		Vector<bool> group_visibility;
		Vector3 copy_buffer;
	} ui_info;

	UndoRedo *undo_redo = nullptr;

	void _add_keyframe(Ref<EPASKeyframe> p_keyframe);
	void _remove_keyframe(Ref<EPASKeyframe> p_keyframe);
	void _rebuild_keyframe_cache();
	void _draw_ui();
	void _draw_bone_positions();
	void _update_editing_handle_trf();
	void _world_to_bone_trf(int p_bone_idx, const float *p_world_trf, Transform3D &p_out);
	void _set_frame_time(int p_frame_idx, int32_t p_frame_time);
	void _create_eirteam_humanoid_ik();
	void _apply_handle_transform(ImGuizmo::OPERATION p_operation);
	void _apply_constraints();
	bool _is_playing();
	void _show_error(const String &error);
	void _add_warp_point(Ref<EPASWarpPoint> p_warp_point);
	void _remove_warp_point(Ref<EPASWarpPoint> p_warp_point);
	bool warp_point_sphere_update_queued = false;
	void queue_warp_point_sphere_update();
	void _load_placeholder_scene(const String &p_path);

	Ref<EPASPose> get_current_pose() const;
	AnimationKeyframeCache *get_keyframe(int p_frame_time) const;

	String current_error;

protected:
	void _notification(int p_what);
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;

public:
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