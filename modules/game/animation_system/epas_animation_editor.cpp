#ifdef DEBUG_ENABLED
#include "epas_animation_editor.h"
#include "core/error/error_macros.h"
#include "core/io/resource_loader.h"
#include "core/math/color.h"
#include "core/object/callable_method_pointer.h"
#include "core/os/memory.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_neo_sequencer.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_animation_node.h"
#include "modules/game/animation_system/epas_pose.h"
#include "modules/game/resources/game_tools_theme.h"
#include "modules/game/utils.h"
#include "modules/imgui/godot_imgui.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/light_3d.h"
#include "scene/3d/world_environment.h"
#include "scene/resources/packed_scene.h"
#include "scene/resources/primitive_meshes.h"
#include "scene/resources/sky_material.h"

Transform3D EPASAnimationEditor::get_edited_object_transform() {
	return Transform3D();
}

void EPASAnimationEditor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			label_font = get_theme_font(SNAME("font"));
			GodotImGui::get_singleton()->set_enable_overlay(false);
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			_draw_ui();
			if (!_is_playing()) {
				epas_animation_node->seek(current_frame / (float)FPS);
			}
		} break;
		case NOTIFICATION_DRAW: {
			selection_handle_dots->hide();
			if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
				selection_handle_dots->show();
				_draw_bone_positions();
			}
		} break;
	}
}

void EPASAnimationEditor::_draw_bone_positions() {
	Camera3D *cam = get_viewport()->get_camera_3d();

	if (cam && editing_skeleton && get_current_pose().is_valid()) {
		if (editing_skeleton->get_bone_count() == 0) {
			return;
		}
		HashMap<int, Vector2> handles_screen_position;
		handles_screen_position.reserve(selection_handles.size());
		Vector2 mouse_pos = get_global_mouse_position();
		int closest_handle = -1;
		Vector2 closest_handle_scr_pos;
		for (int i = 0; i < selection_handles.size(); i++) {
			Vector3 handle_pos;
			if (selection_handles[i]->hidden) {
				continue;
			}
			int bone_idx = selection_handles[i]->bone_idx;
			handle_pos = editing_skeleton->to_global(editing_skeleton->get_bone_global_pose(bone_idx).origin);

			if (!cam->is_position_behind(handle_pos)) {
				Vector2 handle_scr_pos = cam->unproject_position(handle_pos);
				if (mouse_pos.distance_to(handle_scr_pos) < 25.0) {
					if (closest_handle == -1 || mouse_pos.distance_to(handle_scr_pos) < mouse_pos.distance_to(closest_handle_scr_pos)) {
						closest_handle = i;
						closest_handle_scr_pos = handle_scr_pos;
					}
				}

				handles_screen_position.insert(i, handle_scr_pos);
			}
		}
		Color color_unlit = Color::named("Dark Magenta");
		Color color_lit = Color::named("Light Yellow");
		Color color_right = Color::named("Red");
		Color color_left = Color::named("Lime Green");
		Color color_ik = Color::named("Blue");

		Ref<MultiMesh> mm = selection_handle_dots->get_multimesh();

		mm->set_instance_count(handles_screen_position.size());

		int i = 0;

		for (const KeyValue<int, Vector2> &pair : handles_screen_position) {
			String handle_name;
			Color handle_color = color_unlit;
			Ref<Selection> handle = selection_handles[pair.key];
			switch (selection_handles[pair.key]->type) {
				case Selection::FK_BONE: {
					handle_name = editing_skeleton->get_bone_name(selection_handles[pair.key]->bone_idx);
					// Color differentiation for L/R bones
					if (handle_name.ends_with(".L")) {
						handle_color = color_left;
					} else if (handle_name.ends_with(".R")) {
						handle_color = color_right;
					}
				} break;
				case Selection::IK_HANDLE: {
					if (handle->is_ik_magnet) {
						handle_name = handle->ik_joint->get_ik_magnet_bone_name();
					} else {
						handle_name = handle->ik_joint->get_ik_target_bone_name();
					}
					handle_color = color_ik;
				} break;
			}

			if (pair.key == closest_handle) {
				// Hovered handles get their own color
				Vector2 label_pos = pair.value + Vector2(10.0, 0.0);
				draw_string(label_font, label_pos + Vector2(2.0, 2.0), handle_name, HORIZONTAL_ALIGNMENT_LEFT, -1, Font::DEFAULT_FONT_SIZE, Color());
				draw_string(label_font, label_pos, handle_name);
				Transform2D trf;
				trf.set_origin(pair.value);
				mm->set_instance_transform_2d(i, trf);
				mm->set_instance_color(i, color_lit);
			} else {
				Transform2D trf;
				trf.set_origin(pair.value);
				mm->set_instance_transform_2d(i, trf);
				mm->set_instance_color(i, handle_color);
			}
			i++;
		}
		currently_hovered_selection_handle = closest_handle;
	}
}

void EPASAnimationEditor::_world_to_bone_trf(int p_bone_idx, const float *p_world_trf, Transform3D &p_out) {
	HBUtils::mat_to_trf(p_world_trf, p_out);
	p_out = editing_skeleton->get_global_transform().affine_inverse() * p_out;
	int parent = editing_skeleton->get_bone_parent(p_bone_idx);
	if (parent != -1) {
		p_out = editing_skeleton->get_bone_global_pose(parent).affine_inverse() * p_out;
	}
}

// Need this thing for undo_redo reasons
void EPASAnimationEditor::_set_frame_time(int p_frame_idx, int32_t p_frame_time) {
	keyframe_cache[p_frame_idx]->frame_time = p_frame_time;
	keyframe_cache[p_frame_idx]->temporary_frame_time = p_frame_time;
	keyframe_cache[p_frame_idx]->apply_frame_time();
}

void EPASAnimationEditor::_create_eirteam_humanoid_ik() {
	if (!editing_skeleton) {
		_show_error("You need to load a model before creating an IK rig for it!");
		return;
	}
	undo_redo->clear_history();
	Vector<String> ik_tips;
	ik_tips.push_back("hand.L");
	ik_tips.push_back("hand.R");
	ik_tips.push_back("foot.L");
	ik_tips.push_back("foot.R");

	Vector<int> ik_tips_idx;

	for (int i = 0; i < ik_tips.size(); i++) {
		int tip_idx = editing_skeleton->find_bone(ik_tips[i]);
		ERR_FAIL_COND_MSG(tip_idx == -1, vformat("Can't build IK constraints, bone %s does not exist!", ik_tips[i]));
		ik_tips_idx.push_back(tip_idx);
	}

	for (int i = 0; i < selection_handles.size(); i++) {
		if (ik_tips_idx.has(selection_handles[i]->bone_idx)) {
			selection_handles.get(i)->hidden = true;
		}
	}

	for (int i = 0; i < ik_tips.size(); i++) {
		// Create the virtual bones
		String ik_target_bone_name = "IK." + ik_tips[i];
		int target_bone_idx = editing_skeleton->find_bone(ik_target_bone_name);

		if (target_bone_idx == -1) {
			editing_skeleton->add_bone(ik_target_bone_name);
			target_bone_idx = editing_skeleton->get_bone_count() - 1;
		}

		String ik_magnet_bone_name = "IK.Magnet." + ik_tips[i];
		int magnet_bone_idx = editing_skeleton->find_bone(ik_magnet_bone_name);

		if (magnet_bone_idx == -1) {
			editing_skeleton->add_bone(ik_magnet_bone_name);
			magnet_bone_idx = editing_skeleton->get_bone_count() - 1;
		}

		int ik_a_bone_idx = ik_tips_idx[i];
		int ik_b_bone_idx = editing_skeleton->get_bone_parent(ik_a_bone_idx);
		int ik_c_bone_idx = editing_skeleton->get_bone_parent(ik_b_bone_idx);
		{
			// Set the rest position for the target bone
			editing_skeleton->set_bone_rest(target_bone_idx, editing_skeleton->get_bone_global_rest(ik_a_bone_idx));
			editing_skeleton->reset_bone_pose(target_bone_idx);

			Transform3D a_trf = editing_skeleton->get_bone_global_rest(ik_a_bone_idx);
			Transform3D b_trf = editing_skeleton->get_bone_global_rest(ik_b_bone_idx);
			Transform3D c_trf = editing_skeleton->get_bone_global_rest(ik_c_bone_idx);

			Vector3 a_to_b = b_trf.origin - a_trf.origin;
			Vector3 a_to_c = c_trf.origin - a_trf.origin;

			// Use dot product to find out the position of the projection of the middle bone on the hypothenuse (a to c)
			Vector3 a_to_c_magnet_proj = a_to_c.normalized() * (a_to_b.dot(a_to_c) / a_to_c.length());

			const float MAGNET_DISTANCE = 0.5f;
			Vector3 magnet_pos = b_trf.origin + (a_trf.origin + a_to_c_magnet_proj).direction_to(b_trf.origin) * MAGNET_DISTANCE;

			// Set rest position for the magnet bone
			Transform3D magnet_trf;
			magnet_trf.origin = magnet_pos;
			editing_skeleton->set_bone_rest(magnet_bone_idx, magnet_trf);
			editing_skeleton->reset_bone_pose(magnet_bone_idx);
		}

		// Create the constraint
		Ref<EPASAnimationEditorIKJoint> ik_constraint;
		ik_constraint.instantiate();
		ik_constraint->set_ik_magnet_bone_name(ik_magnet_bone_name);
		ik_constraint->set_ik_target_bone_name(ik_target_bone_name);
		ik_constraint->set_ik_tip_bone_name(ik_tips[i]);
		ik_constraint->set_use_magnet(true);

		// Create selection handles
		Ref<Selection> ik_target_handle;
		ik_target_handle.instantiate();
		Ref<Selection> ik_magnet_handle;
		ik_magnet_handle.instantiate();

		ik_target_handle->bone_idx = target_bone_idx;
		ik_target_handle->ik_joint = ik_constraint;
		ik_target_handle->type = Selection::SelectionType::IK_HANDLE;

		ik_magnet_handle->bone_idx = magnet_bone_idx;
		ik_magnet_handle->is_ik_magnet = true;
		ik_magnet_handle->ik_joint = ik_constraint;
		ik_magnet_handle->type = Selection::SelectionType::IK_HANDLE;

		selection_handles.push_back(ik_target_handle);
		selection_handles.push_back(ik_magnet_handle);
		ik_constraints.push_back(ik_constraint);
	}
}

void EPASAnimationEditor::_apply_handle_transform(int p_handle, const Transform3D &p_trf) {
	// Apply the handle transform based on the user's input, after the user is done use
	// _commit_handle_transform to save it to undo_redo
	ERR_FAIL_INDEX_MSG(p_handle, selection_handles.size(), "Handle idx is out of range");
	Ref<Selection> selection_handle = selection_handles[p_handle];
	Ref<EPASPose> current_pose = get_current_pose();
	ERR_FAIL_COND(!current_pose.is_valid());

	Ref<EPASPose> old_pose = current_pose->duplicate();
	int bone_idx = selection_handle->bone_idx;

	// Apply bone transform
	Transform3D bone_trf;
	_world_to_bone_trf(bone_idx, current_handle_trf_matrix.ptr(), bone_trf);
	String bone_name = editing_skeleton->get_bone_name(bone_idx);
	if (!current_pose->get_bone_data(bone_name)) {
		current_pose->create_bone(bone_name);
	}
	switch (guizmo_operation) {
		case ImGuizmo::TRANSLATE: {
			current_pose->set_bone_position(bone_name, bone_trf.origin);
		} break;
		case ImGuizmo::ROTATE: {
			current_pose->set_bone_rotation(bone_name, bone_trf.basis.get_rotation_quaternion());
		} break;
		case ImGuizmo::SCALE: {
			current_pose->set_bone_scale(bone_name, bone_trf.basis.get_scale());
		} break;
		default: {
		};
	}

	_apply_constraints();
}

void EPASAnimationEditor::_apply_constraints() {
	if (!editing_skeleton) {
		return;
	}
	Ref<EPASPose> current_pose = get_current_pose();
	if (!current_pose.is_valid()) {
		return;
	}
	for (int i = 0; i < ik_constraints.size(); i++) {
		Ref<EPASAnimationEditorIKJoint> ik_constraint = ik_constraints[i];

		int target_bone_idx = editing_skeleton->find_bone(ik_constraint->get_ik_target_bone_name());
		int magnet_bone_idx = editing_skeleton->find_bone(ik_constraint->get_ik_magnet_bone_name());

		int c_bone_idx = editing_skeleton->find_bone(ik_constraint->get_ik_tip_bone_name());
		int b_bone_idx = editing_skeleton->get_bone_parent(c_bone_idx);
		int a_bone_idx = editing_skeleton->get_bone_parent(b_bone_idx);

		editing_skeleton->reset_bone_pose(c_bone_idx);
		editing_skeleton->reset_bone_pose(b_bone_idx);
		editing_skeleton->reset_bone_pose(a_bone_idx);

		ERR_FAIL_COND_MSG(c_bone_idx == -1 || b_bone_idx == -1 || a_bone_idx == -1, "Invalid IK constraint");

		Transform3D a_bone_trf = editing_skeleton->get_bone_global_pose(a_bone_idx);
		Transform3D b_bone_trf = editing_skeleton->get_bone_global_pose(b_bone_idx);
		Transform3D c_bone_trf = editing_skeleton->get_bone_global_pose(c_bone_idx);
		Transform3D target_bone_trf = editing_skeleton->get_bone_global_pose(target_bone_idx);

		Quaternion a_local_rot = editing_skeleton->get_bone_pose_rotation(a_bone_idx);
		Quaternion b_local_rot = editing_skeleton->get_bone_pose_rotation(b_bone_idx);

		HBUtils::two_joint_ik(
				a_bone_trf.origin,
				b_bone_trf.origin,
				c_bone_trf.origin,
				target_bone_trf.origin, 0.01f,
				a_bone_trf.basis.get_rotation_quaternion(),
				b_bone_trf.basis.get_rotation_quaternion(),
				a_local_rot,
				b_local_rot,
				true,
				// Magnet is a child of no one, so its ok to use the local origin here
				editing_skeleton->get_bone_pose_position(magnet_bone_idx));

		String a_bone_name = editing_skeleton->get_bone_name(a_bone_idx);
		String b_bone_name = editing_skeleton->get_bone_name(b_bone_idx);
		String c_bone_name = ik_constraint->get_ik_tip_bone_name();

		const int bone_indices[] = { a_bone_idx, b_bone_idx };
		const Quaternion *bone_rots[] = { &a_local_rot, &b_local_rot };

		for (int j = 0; j < 2; j++) {
			String bone_name = editing_skeleton->get_bone_name(bone_indices[j]);
			if (!current_pose->has_bone(bone_name)) {
				current_pose->create_bone(bone_name);
			}
			current_pose->set_bone_rotation(bone_name, *bone_rots[j]);
		}

		if (!current_pose->has_bone(c_bone_name)) {
			current_pose->create_bone(c_bone_name);
		}
		// Align the tip of rotation
		Transform3D b_global_trf = current_pose->calculate_bone_global_transform(b_bone_name, editing_skeleton, epas_controller->get_base_pose());
		Quaternion c_rot = b_global_trf.basis.get_rotation_quaternion().inverse() * target_bone_trf.basis.get_rotation_quaternion();
		current_pose->set_bone_rotation(c_bone_name, c_rot);
	}
}

Ref<EPASPose> EPASAnimationEditor::get_current_pose() const {
	// Returns the current pose that the user is hovering, if any
	AnimationKeyframeCache *kf = get_keyframe(current_frame);
	if (kf) {
		return kf->keyframe->get_pose();
	}
	return Ref<EPASPose>();
}

EPASAnimationEditor::AnimationKeyframeCache *EPASAnimationEditor::get_keyframe(int p_frame_time) const {
	for (int i = 0; i < keyframe_cache.size(); i++) {
		if (keyframe_cache[i]->frame_time == p_frame_time) {
			return keyframe_cache[i];
		}
	}
	return nullptr;
}

void EPASAnimationEditor::_add_keyframe(Ref<EPASKeyframe> p_keyframe) {
	AnimationKeyframeCache *kc = memnew(AnimationKeyframeCache(p_keyframe));
	keyframe_cache.push_back(kc);
	current_animation->add_keyframe(p_keyframe);
}

void EPASAnimationEditor::_remove_keyframe(Ref<EPASKeyframe> p_keyframe) {
	for (int i = 0; i < keyframe_cache.size(); i++) {
		if (keyframe_cache[i]->keyframe == p_keyframe) {
			keyframe_cache.erase(keyframe_cache[i]);
			break;
		}
	}
	current_animation->erase_keyframe(p_keyframe);
}

void EPASAnimationEditor::_rebuild_keyframe_cache() {
	for (int i = 0; i < keyframe_cache.size(); i++) {
		memdelete(keyframe_cache[i]);
	}
	keyframe_cache.clear();

	ERR_FAIL_COND(!current_animation.is_valid());

	keyframe_cache.resize(current_animation->get_keyframe_count());
	for (int i = 0; i < current_animation->get_keyframe_count(); i++) {
		AnimationKeyframeCache *kc = memnew(AnimationKeyframeCache(current_animation->get_keyframe(i)));
		keyframe_cache.set(i, kc);
	}
}

void EPASAnimationEditor::_draw_ui() {
	ImGuiIO io = ImGui::GetIO();
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Open", "CTRL+O")) {
				file_open_dialog->popup_centered_ratio(0.75);
			}
			if (ImGui::MenuItem("Save", "CTRL+S", false, !current_animation->get_path().is_empty())) {
				save_to_path(current_animation->get_path());
			}
			if (ImGui::MenuItem("Save as...", "CTRL+S", false)) {
				save_file_dialog->popup_centered_ratio(0.75);
			}
			ImGui::Separator();
			if (ImGui::MenuItem("Load model", "CTRL+L")) {
				model_load_dialog->popup_centered_ratio(0.75);
			}
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Tools")) {
			if (ImGui::MenuItem("Flip pose")) {
				Ref<EPASPose> current_pose = get_current_pose();
				if (current_pose.is_valid()) {
					undo_redo->create_action("Flip pose");
					undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::flip_along_z));
					undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::flip_along_z));
					undo_redo->commit_action();
				}
			}
			if (ImGui::MenuItem("Create EIRTeam humanoid IK rig")) {
				_create_eirteam_humanoid_ik();
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
		ImVec2 window_pos;
		ImVec2 work_pos = ImGui::GetMainViewport()->WorkPos;
		window_pos.x = work_pos.x + 5.0f;
		window_pos.y = work_pos.y + 5.0f;
		ImGui::SetNextWindowBgAlpha(0.0);
		ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, ImVec2(0.0f, 0.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2());
		ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize;
		if (ImGui::Begin("ToolDock", nullptr, window_flags)) {
			ImGui::SetNextItemWidth(70);
			ImGui::Combo("", (int *)&guizmo_mode, "Local\0Global\0\0");
			ImGui::BeginDisabled(guizmo_operation == ImGuizmo::OPERATION::TRANSLATE);
			if (ImGui::Button(FONT_REMIX_ICON_DRAG_MOVE_2_LINE)) {
				guizmo_operation = ImGuizmo::TRANSLATE;
			}
			ImGui::EndDisabled();
			ImGui::BeginDisabled(guizmo_operation == ImGuizmo::OPERATION::ROTATE);
			if (ImGui::Button(FONT_REMIX_ICON_CLOCKWISE_LINE)) {
				guizmo_operation = ImGuizmo::ROTATE;
			}
			ImGui::EndDisabled();
		}
		ImGui::End();
		ImGui::PopStyleVar();
	}

	/*
	ImGuiID dock_id = ImGui::GetID("pose_dock");
	ImGuiID bottom_id = ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Down, 0.2f, nullptr, &dock_id);
	ImGui::DockBuilderDockWindow("Timeline", bottom_id);*/
	ImGuiID dock_id = ImGui::GetID("pose_dock");
	const ImGuiViewport *viewport = ImGui::GetMainViewport();
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
	flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus;
	flags |= ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground;
	ImGui::SetNextWindowPos(viewport->WorkPos);
	ImGui::SetNextWindowSize(viewport->WorkSize);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::Begin("MainNode", nullptr, flags);
	if (!ImGui::DockBuilderGetNode(dock_id)) {
		ImGui::DockBuilderRemoveNode(dock_id);
		ImGui::DockBuilderAddNode(dock_id, ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
		ImGui::DockBuilderSetNodeSize(dock_id, viewport->WorkSize);
		ImGuiID dock_up_r;
		ImGuiID dock_bottom = ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Down, 0.3f, nullptr, &dock_up_r);
		dock_up_r = ImGui::DockBuilderSplitNode(dock_up_r, ImGuiDir_Right, 0.2f, nullptr, nullptr);

		ImGui::DockBuilderDockWindow("Timeline", dock_bottom);
		ImGui::DockBuilderDockWindow("Settings", dock_up_r);
		ImGui::DockBuilderDockWindow("PoseDBG", dock_up_r);
		ImGui::DockBuilderDockWindow("ConstraintDBG", dock_up_r);
		ImGui::DockBuilderFinish(dock_id);
	}
	ImGui::DockSpace(dock_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
	ImGui::PopStyleVar();
	ImGui::End();

	if (_is_playing()) {
		current_frame = epas_animation_node->get_time() * FPS;
	}

	if (ImGui::Begin("Timeline", nullptr, ImGuiWindowFlags_NoTitleBar)) {
		Ref<EPASPose> current_pose = get_current_pose();
		ImGui::BeginDisabled(current_pose.is_valid() || _is_playing());
		if (ImGui::Button(FONT_REMIX_ICON_ADD)) {
			// Add keyframe
			if (current_pose.is_valid()) {
				// Get the currently interpolated pose, if we don't have any just use an empty pose
				Ref<EPASPose> output_pose = epas_controller->get_output_pose();
				Ref<EPASPose> kf_pose = output_pose.is_valid() ? output_pose->duplicate() : memnew(EPASPose);

				undo_redo->create_action("Add new keyframe");
				Ref<EPASKeyframe> kf;
				kf.instantiate();
				kf->set_pose(kf_pose);
				kf->set_time(current_frame / (float)FPS);
				undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_add_keyframe).bind(kf));
				undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_remove_keyframe).bind(kf));
				undo_redo->commit_action();
			}
		}
		ImGui::EndDisabled();
		ImGui::SameLine();
		ImGui::BeginDisabled(!current_pose.is_valid() || _is_playing());
		if (ImGui::Button(FONT_REMIX_ICON_SUBTRACT)) {
			// Remove keyframe
			AnimationKeyframeCache *kf = get_keyframe(current_frame);
			if (kf) {
				undo_redo->create_action("Remove keyframe");
				undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_remove_keyframe).bind(kf->keyframe));
				undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_add_keyframe).bind(kf->keyframe));
				undo_redo->commit_action();
			}
		}
		ImGui::EndDisabled();
		bool has_selection = kf_selection.size() != 0;
		ImGui::SameLine();
		ImGui::BeginDisabled(!has_selection);
		if (ImGui::Button(FONT_REMIX_ICON_FILE_COPY_LINE)) {
			// duplicate frame at current position
			if (!get_current_pose().is_valid() || _is_playing()) {
				// Todo: make this actually copy more than one keyframe...
				AnimationKeyframeCache *kfc = get_keyframe(kf_selection[0]);
				if (kfc) {
					undo_redo->create_action("Duplicate keyframe");
					Ref<EPASKeyframe> kf = kfc->keyframe->duplicate(true);
					kf->set_time(current_frame / (float)FPS);
					undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_remove_keyframe).bind(kf));
					undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_add_keyframe).bind(kf));
					undo_redo->commit_action();
				}
			}
		}
		ImGui::EndDisabled();
		ImGui::SameLine();
		ImGui::Spacing();
		ImGui::SameLine();
		if (_is_playing()) {
			if (ImGui::Button(FONT_REMIX_ICON_PAUSE_LINE)) {
				epas_animation_node->set_playback_mode(EPASAnimationNode::MANUAL);
			}
		} else if (ImGui::Button(FONT_REMIX_ICON_PLAY_LINE)) {
			epas_animation_node->set_playback_mode(EPASAnimationNode::AUTOMATIC);
		}

		ImGuiNeoSequencerFlags sequencer_flags = ImGuiNeoSequencerFlags_EnableSelection;
		sequencer_flags |= ImGuiNeoSequencerFlags_Selection_EnableDragging;
		sequencer_flags |= ImGuiNeoSequencerFlags_Selection_EnableDeletion;
		if (_is_playing()) {
			sequencer_flags = 0;
		}
		int32_t old_current_frame = current_frame;
		if (ImGui::BeginNeoSequencer("Timeline", &current_frame, &start_frame, &end_frame, ImVec2(), sequencer_flags)) {
			if (ImGui::BeginNeoTimelineEx("Keyframes", nullptr, ImGuiNeoTimelineFlags_AllowFrameChanging)) {
				bool has_changed_frames = false;
				for (int i = 0; i < keyframe_cache.size(); i++) {
					AnimationKeyframeCache *kc = keyframe_cache[i];
					ImGui::NeoKeyframe(&kc->temporary_frame_time);

					// if a frame position's has changed, we must update the time in the animation system
					if (kc->temporary_frame_time != kc->frame_time) {
						if (ImGui::NeoIsDraggingSelection()) {
							kc->apply_temp_frame_time();
						} else {
							if (!has_changed_frames) {
								has_changed_frames = true;
								undo_redo->create_action("Move keyframe");
							}
							undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_set_frame_time).bind(i, kc->frame_time));
							undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_set_frame_time).bind(i, kc->temporary_frame_time));
						}
					}
				}
				if (has_changed_frames) {
					undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
					undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
					undo_redo->commit_action();
				}

				if (ImGui::NeoHasSelection()) {
					kf_selection.resize(ImGui::GetNeoKeyframeSelectionSize());
					ImGui::GetNeoKeyframeSelection(kf_selection.ptrw());
				} else {
					kf_selection.clear();
				}

				ImGui::EndNeoTimeLine();
			}
			ImGui::EndNeoSequencer();
		}

		// Frame changed, hide the bone selector gizmos!
		if (old_current_frame != current_frame) {
			queue_redraw();
		}
	}
	ImGui::End();

	if (ImGui::Begin("Settings")) {
		EPASAnimation::InterpolationMethod interp_method = epas_animation_node->get_interpolation_method();
		EPASAnimation::InterpolationMethod original_interp_method = interp_method;
		const char *items[] = { "Step", "Linear", "Bicubic" };
		ImGui::SetNextItemWidth(80);
		ImGui::Combo("Interpolation", reinterpret_cast<int *>(&interp_method), items, IM_ARRAYSIZE(items));
		if (interp_method != original_interp_method) {
			epas_animation_node->set_interpolation_method(interp_method);
		}
		ImGui::InputInt("Length", &end_frame);
	};
	ImGui::End();

	Ref<EPASPose> current_pose = get_current_pose();

	if (ImGui::Begin("PoseDBG")) {
		if (current_pose.is_valid()) {
			for (const KeyValue<String, EPASPose::BoneData *> &ps : current_pose->get_bone_map()) {
				if (ImGui::CollapsingHeader(ps.value->bone_name.utf8().get_data())) {
					ImGui::PushID(ps.value);
					if (ps.value->has_position) {
						ImGui::AlignTextToFramePadding();
						ImGui::TextUnformatted(vformat("Pos %s", ps.value->position).utf8().ptr());
						ImGui::SameLine();
						if (ImGui::Button("X##pos")) {
							undo_redo->create_action("Remove position from keyframe");
							undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_position).bind(ps.key, false));
							undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_position).bind(ps.key, true));
							undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
							undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
							undo_redo->commit_action();
						}
					}
					if (ps.value->has_rotation) {
						ImGui::AlignTextToFramePadding();
						ImGui::TextUnformatted(vformat("Rot %s", ps.value->rotation).utf8().ptr());
						ImGui::SameLine();
						if (ImGui::Button("X##rot")) {
							undo_redo->create_action("Remove rotation from keyframe");
							undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_rotation).bind(ps.key, false));
							undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_rotation).bind(ps.key, true));
							undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
							undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
							undo_redo->commit_action();
						}
					}
					if (ps.value->has_scale) {
						ImGui::AlignTextToFramePadding();
						ImGui::TextUnformatted(vformat("Scale %s", ps.value->scale).utf8().ptr());
						ImGui::SameLine();
						if (ImGui::Button("X##scale")) {
							undo_redo->create_action("Remove scale from keyframe");
							undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_scale).bind(ps.key, false));
							undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_scale).bind(ps.key, true));
							undo_redo->commit_action();
						}
					}
					ImGui::PopID();
				}
			}
		}
	}
	ImGui::End();
	if (ImGui::Begin("ConstraintDBG")) {
		for (int i = 0; i < ik_constraints.size(); i++) {
			if (editing_skeleton) {
				if (ImGui::CollapsingHeader(vformat("Constraint %d", i).utf8().get_data(), ImGuiTreeNodeFlags_DefaultOpen)) {
					String tip_bone_name = ik_constraints[i]->get_ik_tip_bone_name();
					ImGui::Text("Tip bone: %s (%d)", tip_bone_name.utf8().get_data(), editing_skeleton->find_bone(tip_bone_name));
					String target_bone_name = ik_constraints[i]->get_ik_target_bone_name();
					ImGui::Text("Target bone: %s (%d)", target_bone_name.utf8().get_data(), editing_skeleton->find_bone(target_bone_name));
					String magnet_bone_name = ik_constraints[i]->get_ik_magnet_bone_name();
					ImGui::Text("Magnet bone: %s (%d)", magnet_bone_name.utf8().get_data(), editing_skeleton->find_bone(magnet_bone_name));
				}
			}
		}
	}
	ImGui::End();

	Ref<World3D> world = editor_3d_root->get_world_3d();

	bool manipulated = false;
	if (world.is_valid()) {
		Camera3D *cam = get_viewport()->get_camera_3d();
		if (!cam) {
			return;
		}

		Transform3D view_trf = cam->get_global_transform().inverse();
		HBUtils::trf_to_mat(view_trf, view_matrix.ptrw());

		Projection proj;
		float aspect = io.DisplaySize.x / io.DisplaySize.y;
		proj.set_perspective(cam->get_fov(), aspect, cam->get_near(), cam->get_far(), cam->get_keep_aspect_mode() == Camera3D::KEEP_WIDTH);
		HBUtils::proj_to_mat(proj, projection_matrix.ptrw());

		ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
		if (editing_selection_handle != -1) {
			const Ref<Selection> selection_handle = selection_handles[editing_selection_handle];
			manipulated = ImGuizmo::Manipulate(view_matrix.ptr(), projection_matrix.ptr(), guizmo_operation, guizmo_mode, current_handle_trf_matrix.ptrw());
			if (manipulated) {
				// Apply handle transform, without committing it to undo_redo
				Transform3D trf;
				HBUtils::mat_to_trf(current_handle_trf_matrix.ptr(), trf);
				_apply_handle_transform(editing_selection_handle, trf);
			}
			if (!ImGuizmo::IsUsing() && was_using_gizmo) {
				_commit_guizmo_transform();
			}
			was_using_gizmo = ImGuizmo::IsUsing();
		}
	}

	// Draw error modal
	if (!current_error.is_empty() && !ImGui::IsPopupOpen("Error")) {
		ImGui::OpenPopup("Error");
	}
	if (ImGui::BeginPopupModal("Error", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::TextUnformatted(current_error.utf8().ptr());
		if (ImGui::Button("Ok", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
			current_error.clear();
			ImGui::CloseCurrentPopup();
		}
		ImGui::SetItemDefaultFocus();
		ImGui::EndPopup();
	}
}

void EPASAnimationEditor::_commit_guizmo_transform() {
	// Commit current imguizmo transform to undo_redo
	if (editing_selection_handle == -1) {
		return;
	}
	Ref<EPASPose> current_pose = get_current_pose();

	if (!current_pose.is_valid()) {
		return;
	}

	const Ref<Selection> selection_handle = selection_handles[editing_selection_handle];
	// No need to get fancy since both fk_bone_idx and ik_bone_idx are ints
	int editing_bone = selection_handle->bone_idx;
	String bone_name = editing_skeleton->get_bone_name(editing_bone);
	Transform3D new_bone_trf;
	Transform3D old_bone_trf;
	_world_to_bone_trf(editing_bone, current_handle_trf_matrix.ptr(), new_bone_trf);
	_world_to_bone_trf(editing_bone, prev_handle_trf_matrix.ptr(), old_bone_trf);
	if (!current_pose->get_bone_data(bone_name)) {
		current_pose->create_bone(bone_name);
	}
	switch (guizmo_operation) {
		case ImGuizmo::TRANSLATE: {
			undo_redo->create_action("Translate bone");
			undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind(bone_name, current_pose->get_bone_position(bone_name)));
			undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind(bone_name, old_bone_trf.origin));
		} break;
		case ImGuizmo::ROTATE: {
			undo_redo->create_action("Rotate bone");
			undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_rotation).bind(bone_name, current_pose->get_bone_rotation(bone_name)));
			undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_rotation).bind(bone_name, old_bone_trf.basis.get_rotation_quaternion()));

		} break;
		case ImGuizmo::SCALE: {
			undo_redo->create_action("Scale bone");
			undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_scale).bind(bone_name, current_pose->get_bone_scale(bone_name)));
			undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_scale).bind(bone_name, old_bone_trf.basis.get_scale()));
		} break;
		default: {
		};
	}
	undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_update_editing_handle_trf));
	undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_update_editing_handle_trf));
	undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
	undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints));
	undo_redo->commit_action();
}

bool EPASAnimationEditor::_is_playing() {
	return epas_animation_node->get_playback_mode() == EPASAnimationNode::AUTOMATIC;
}

void EPASAnimationEditor::_show_error(const String &error) {
	current_error = error;
}

void EPASAnimationEditor::open_file(const String &p_path) {
	Error err;
	Ref<EPASAnimation> animation = ResourceLoader::load(p_path, "EPASAnimation", ResourceFormatLoader::CACHE_MODE_REUSE, &err);
	print_line("Err", err);
	if (err == OK) {
		if (animation.is_valid()) {
			undo_redo->clear_history();
			if (animation->has_meta("__editor_model_path")) {
				String model_path = animation->get_meta("__editor_model_path");
				load_model(model_path);
			}
			set_animation(animation);
			if (animation->get_meta("__editor_humanoid_ik", false)) {
				_create_eirteam_humanoid_ik();
			}
		} else {
			_show_error("Selected file was not an EPAS animation file.");
		}
	} else {
		_show_error("Error loading animation: " + String(error_names[err]));
	}
}

void EPASAnimationEditor::load_model(const String &path) {
	undo_redo->clear_history();
	if (current_model) {
		current_model->queue_free();
		current_model = nullptr;
		editing_skeleton = nullptr;
	}

	Ref<PackedScene> scene = ResourceLoader::load(path);
	if (scene.is_null()) {
		_show_error("Loaded file was not a PackedScene!");
	}

	Node *node = scene->instantiate();
	Node3D *node_3d = Object::cast_to<Node3D>(node);
	if (!node_3d) {
		if (node) {
			node->queue_free();
		}
		_show_error("Model was not a 3D scene");
		return;
	}
	TypedArray<Node> skel_candidate = node_3d ? node_3d->find_children("*", "Skeleton3D") : TypedArray<Node>();
	Skeleton3D *skel = nullptr;
	if (skel_candidate.size() > 0) {
		skel = Object::cast_to<Skeleton3D>(skel_candidate[0]);
	}
	if (!skel) {
		if (node) {
			node->queue_free();
		}
		_show_error("Model did not contain any skeleton!");
		return;
	}

	add_child(node_3d);
	epas_controller->set_skeleton_path(skel->get_path());
	editing_skeleton = skel;
	current_model = node_3d;

	// Add selection handles
	selection_handles.clear();
	for (int i = 0; i < skel->get_bone_count(); i++) {
		Ref<Selection> selection_handle;
		selection_handle.instantiate();
		selection_handle->bone_idx = i;
		selection_handles.push_back(selection_handle);
	}
}

void EPASAnimationEditor::_update_editing_handle_trf() {
	if (editing_selection_handle != -1) {
		Transform3D handle_trf;
		const Ref<Selection> handle = selection_handles[editing_selection_handle];
		handle_trf = editing_skeleton->get_bone_global_pose(handle->bone_idx);
		handle_trf = editing_skeleton->get_global_transform() * handle_trf;
		HBUtils::trf_to_mat(handle_trf, current_handle_trf_matrix.ptrw());
		prev_handle_trf_matrix = current_handle_trf_matrix.duplicate();
		queue_redraw();
	}
}

void EPASAnimationEditor::input(const Ref<InputEvent> &p_event) {
	const Ref<InputEventKey> &kev = p_event;
	if (kev.is_valid()) {
		if (kev->is_pressed() && !kev->is_echo() && kev->get_modifiers_mask().has_flag(KeyModifierMask::CTRL)) {
			if (kev->get_keycode() == Key::Z) {
				undo_redo->undo();
			} else if (kev->get_keycode() == Key::Y) {
				undo_redo->redo();
			}
		}
	}
	const Ref<InputEventMouseMotion> &mev = p_event;
	if (mev.is_valid()) {
		if (mev->get_relative().length() > 0) {
			queue_redraw();
		}
	}
	const Ref<InputEventMouseButton> &bev = p_event;
	if (bev.is_valid()) {
		if (bev->get_button_index() == MouseButton::LEFT) {
			if (!bev->is_pressed()) {
				editing_selection_handle = -1;
				if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
					editing_selection_handle = currently_hovered_selection_handle;
					if (editing_selection_handle != -1) {
						_update_editing_handle_trf();
						accept_event();
					}
				}
			}
		}
	}
}

void EPASAnimationEditor::set_animation(const Ref<EPASAnimation> &p_animation) {
	current_animation = p_animation;
	epas_animation_node->set_animation(p_animation);
	_rebuild_keyframe_cache();
}

Ref<EPASAnimation> EPASAnimationEditor::get_animation() const {
	return current_animation;
}

void EPASAnimationEditor::save_to_path(const String &p_path) {
	ERR_FAIL_COND(current_animation.is_null());
	if (current_model) {
		current_animation->set_meta("__editor_model_path", current_model->get_scene_file_path());
	}
	if (ik_constraints.size() > 0) {
		current_animation->set_meta("__editor_humanoid_ik", true);
	}
	int err_code = ResourceSaver::save(current_animation, p_path, ResourceSaver::FLAG_CHANGE_PATH);
	ERR_FAIL_COND_MSG(err_code != OK, vformat("Error saving animation, %s", error_names[err_code]));
}

EPASAnimationEditor::EPASAnimationEditor() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		// HACK-ish but works
		set_z_index(-100);
		set_theme(GameToolsTheme::generate_theme());
		{
			// Generate the multimesh used for the selection handle dots
			selection_handle_dots = memnew(MultiMeshInstance2D);
			selection_handle_dots->set_texture(get_theme_icon("Dot", "GameTools"));
			const float DOT_RADIUS = 8.0f;
			Ref<QuadMesh> quad;
			quad.instantiate();
			quad->set_size(Vector2(DOT_RADIUS * 2.0f, DOT_RADIUS * 2.0f));

			Ref<MultiMesh> mm;
			mm.instantiate();
			mm->set_use_colors(true);
			mm->set_mesh(quad);
			selection_handle_dots->set_multimesh(mm);

			selection_handle_dots->set_draw_behind_parent(true);
			add_child(selection_handle_dots);
		}
		editor_3d_root = memnew(Node3D);

		add_child(editor_3d_root);
		{
			file_open_dialog = memnew(FileDialog);
			PackedStringArray filters;
			filters.push_back("*.tres ; TRES");
			filters.push_back("*.epan ; EPAS Animation");
			file_open_dialog->set_filters(filters);
			add_child(file_open_dialog);
			file_open_dialog->set_file_mode(FileDialog::FileMode::FILE_MODE_OPEN_FILE);
			file_open_dialog->connect("file_selected", callable_mp(this, &EPASAnimationEditor::open_file));
		}
		{
			model_load_dialog = memnew(FileDialog);
			PackedStringArray filters;
			filters.push_back("*.gltf ; GLTF model");
			filters.push_back("*.blend ; Blender file");
			model_load_dialog->set_filters(filters);
			add_child(model_load_dialog);
			model_load_dialog->set_file_mode(FileDialog::FileMode::FILE_MODE_OPEN_FILE);
			model_load_dialog->connect("file_selected", callable_mp(this, &EPASAnimationEditor::load_model));
		}
		{
			save_file_dialog = memnew(FileDialog);
			PackedStringArray filters;
			filters.push_back("*.tres ; TRES");
			filters.push_back("*.epan ; EPAS Animation");
			save_file_dialog->set_filters(filters);
			add_child(save_file_dialog);
			save_file_dialog->set_file_mode(FileDialog::FileMode::FILE_MODE_SAVE_FILE);
			save_file_dialog->connect("file_selected", callable_mp(this, &EPASAnimationEditor::save_to_path));
		}

		set_process_internal(true);
		// Setup 3d buffer and identity matrix
		view_matrix.resize_zeroed(16);
		projection_matrix.resize_zeroed(16);
		current_handle_trf_matrix.resize_zeroed(16);
		identity_matrix.resize_zeroed(16);
		HBUtils::trf_to_mat(Transform3D(), identity_matrix.ptrw());

		camera = memnew(EPASEditorCamera);
		editor_3d_root->add_child(camera);
		camera->set_position(Vector3(0.0, 1.0, 1.0));

		DirectionalLight3D *dl = memnew(DirectionalLight3D);
		editor_3d_root->add_child(dl);
		dl->rotate_x(Math::deg_to_rad(-45.0));
		dl->rotate_y(Math::deg_to_rad(-45.0));

		grid = memnew(EPASEditorGrid);
		editor_3d_root->add_child(grid);

		{
			WorldEnvironment *wenv = memnew(WorldEnvironment);
			Ref<Environment> env = memnew(Environment);
			Ref<Sky> sky = memnew(Sky);
			Ref<ProceduralSkyMaterial> sky_mat = memnew(ProceduralSkyMaterial);
			env->set_background(Environment::BGMode::BG_SKY);
			env->set_sky(sky);
			sky->set_material(sky_mat);
			sky_mat->set_sky_top_color(Color("#01a1f7"));
			sky_mat->set_sky_horizon_color(Color("#e0c9e6"));
			sky_mat->set_ground_horizon_color(Color("#e0c9e6"));
			sky_mat->set_ground_bottom_color(Color("#7b8cc4"));

			wenv->set_environment(env);
			add_child(wenv);
		}

		set_process_input(true);

		// Setup animation display

		epas_controller = memnew(EPASController);
		add_child(epas_controller, true);

		epas_animation_node = Ref<EPASAnimationNode>(memnew(EPASAnimationNode));
		epas_animation_node->set_playback_mode(EPASAnimationNode::PlaybackMode::MANUAL);
		epas_controller->connect_node_to_root(epas_animation_node, "Animation");

		undo_redo = memnew(UndoRedo);
		set_animation(memnew(EPASAnimation));
	}
}

EPASAnimationEditor::~EPASAnimationEditor() {
	if (undo_redo) {
		memdelete(undo_redo);
	}
	for (int i = 0; i < keyframe_cache.size(); i++) {
		memdelete(keyframe_cache[i]);
	}
	keyframe_cache.clear();
}

#endif