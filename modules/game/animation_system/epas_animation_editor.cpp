/**************************************************************************/
/*  epas_animation_editor.cpp                                             */
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

#include "epas_animation_editor.h"
#include "core/error/error_macros.h"
#include "core/io/resource_loader.h"
#include "core/math/color.h"
#include "core/object/callable_method_pointer.h"
#include "core/os/memory.h"

#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_animation_node.h"
#include "modules/game/animation_system/epas_pose.h"
#include "modules/game/transform_conversions.h"
#include "modules/imgui/godot_imgui.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/light_3d.h"
#include "scene/3d/world_environment.h"
#include "scene/resources/3d/primitive_meshes.h"
#include "scene/resources/3d/sky_material.h"
#include "scene/resources/packed_scene.h"

void EPASEditorAnimation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_editor_animation", "editor_animation"), &EPASEditorAnimation::set_editor_animation);
	ClassDB::bind_method(D_METHOD("get_editor_animation"), &EPASEditorAnimation::get_editor_animation);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "editor_animation", PROPERTY_HINT_RESOURCE_TYPE, "EPASAnimation"), "set_editor_animation", "get_editor_animation");
	ClassDB::bind_method(D_METHOD("set_editor_interpolation_method", "et_editor_interpolation_method"), &EPASEditorAnimation::set_editor_interpolation_method);
	ClassDB::bind_method(D_METHOD("get_editor_interpolation_method"), &EPASEditorAnimation::get_editor_interpolation_method);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "editor_interpolation_method", PROPERTY_HINT_ENUM), "set_editor_interpolation_method", "get_editor_interpolation_method");
}

#ifdef DEBUG_ENABLED

#include "editor/editor_settings.h"
#include "editor/plugins/curve_editor_plugin.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_neo_sequencer.h"
#include "modules/game/resources/game_tools_theme.h"
void EPASAnimationEditor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			Input::get_singleton()->set_mouse_mode(Input::MouseMode::MOUSE_MODE_VISIBLE);
		} break;
		case NOTIFICATION_ENTER_TREE: {
			label_font = get_theme_font(SNAME("font"));
			GodotImGui::get_singleton()->set_enable_overlay(false);
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			if (warp_point_sphere_update_queued) {
				// Update warp point spheres to new positions
				Ref<MultiMesh> mm = warp_point_spheres->get_multimesh();
				mm->set_instance_count(current_animation->get_warp_point_count());
				for (int i = 0; i < current_animation->get_warp_point_count(); i++) {
					Transform3D trf = current_animation->get_warp_point(i)->get_transform();
					mm->set_instance_transform(i, trf);
				}
				warp_point_sphere_update_queued = false;
			}

			_draw_ui();
			if (current_animation.is_valid() && current_animation->get_editor_animation().is_valid()) {
				curves_editor->draw(current_animation, epas_animation_node->get_time() / current_animation->get_editor_animation()->get_length());
				events_editor->draw(current_animation, epas_animation_node->get_time());
			}
			if (!_is_playing()) {
				epas_animation_node->seek(current_frame / (float)FPS);
			}

			epas_controller->advance(get_process_delta_time());
			// Hacky but oh well
			if (ui_info.enable_rt_ik) {
				epas_blend_node->set_blend_amount(0.0f);
				Ref<EPASPose> output_pose = epas_controller->get_output_pose()->duplicate();
				_apply_constraints(output_pose);
				epas_pose_node->set_pose(output_pose);
				epas_blend_node->set_blend_amount(1.0f);
				epas_controller->advance(0.0f);
				epas_blend_node->set_blend_amount(0.0f);
			}
		} break;
		case NOTIFICATION_DRAW: {
			selection_handle_dots->hide();
			if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
				selection_handle_dots->show();
				_draw_bone_positions(false);
			} else if (editing_selection_handles.size() > 0) {
				selection_handle_dots->show();
				_draw_bone_positions(true);
			}
		} break;
	}
}

void EPASAnimationEditor::_draw_bone_positions(bool p_selected_only) {
	Camera3D *cam = get_viewport()->get_camera_3d();

	if (p_selected_only && editing_selection_handles.size() == 0) {
		return;
	}

	if (cam && editing_skeleton && get_current_pose().is_valid()) {
		if (selection_handles.size() == 0 && !p_selected_only) {
			return;
		}
		HashMap<int, Vector2> handles_screen_position;
		handles_screen_position.reserve(selection_handles.size());
		Vector2 mouse_pos = get_global_mouse_position();
		int closest_handle = -1;
		Vector2 closest_handle_scr_pos;
		for (int i = 0; i < selection_handles.size(); i++) {
			Vector3 handle_pos;
			if (p_selected_only && !editing_selection_handles.has(i)) {
				continue;
			}
			if (selection_handles[i]->hidden || !ui_info.group_visibility[selection_handles[i]->group]) {
				continue;
			}
			switch (selection_handles[i]->type) {
				case Selection::SelectionType::FK_BONE:
				case Selection::SelectionType::IK_HANDLE: {
					int bone_idx = selection_handles[i]->bone_idx;
					handle_pos = editing_skeleton->to_global(editing_skeleton->get_bone_global_pose(bone_idx).origin);
				} break;
				case Selection::SelectionType::WARP_POINT: {
					handle_pos = selection_handles[i]->warp_point->get_transform().origin;
				}
			}

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
		Color color_warp_point = Color::named("Green");

		Ref<MultiMesh> mm = selection_handle_dots->get_multimesh();

		mm->set_instance_count(handles_screen_position.size());

		int i = 0;

		for (const KeyValue<int, Vector2> &pair : handles_screen_position) {
			String handle_name;
			Color handle_color = color_unlit;
			Ref<Selection> handle = selection_handles[pair.key];
			if (p_selected_only && !editing_selection_handles.has(pair.key)) {
				continue;
			}
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
				case Selection::WARP_POINT: {
					handle_color = color_warp_point;
					handle_name = handle->warp_point->get_point_name();
				} break;
			}

			if (pair.key == closest_handle || p_selected_only) {
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
	HBTransformConversions::mat_to_trf(p_world_trf, p_out);
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
	int root = editing_skeleton->get_parentless_bones()[0];
	Transform3D root_rest = editing_skeleton->get_bone_rest(root);
	undo_redo->clear_history();
	Vector<String> ik_tips;
	ik_tips.push_back("hand.L");
	ik_tips.push_back("hand.R");
	ik_tips.push_back("foot.L");
	ik_tips.push_back("foot.R");

	Vector<int> bone_handles_to_hide;

	for (int i = 0; i < ik_tips.size(); i++) {
		int tip_idx = editing_skeleton->find_bone(ik_tips[i]);
		ERR_FAIL_COND_MSG(tip_idx == -1, vformat("Can't build IK constraints, bone %s does not exist!", ik_tips[i]));
		bone_handles_to_hide.push_back(tip_idx);
		bone_handles_to_hide.push_back(editing_skeleton->get_bone_parent(tip_idx));
	}

	for (int i = 0; i < selection_handles.size(); i++) {
		if (bone_handles_to_hide.has(selection_handles[i]->bone_idx)) {
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
			editing_skeleton->set_bone_parent(target_bone_idx, root);
		}

		String ik_magnet_bone_name = "IK.Magnet." + ik_tips[i];
		int magnet_bone_idx = editing_skeleton->find_bone(ik_magnet_bone_name);

		if (magnet_bone_idx == -1) {
			editing_skeleton->add_bone(ik_magnet_bone_name);
			magnet_bone_idx = editing_skeleton->get_bone_count() - 1;
			editing_skeleton->set_bone_parent(magnet_bone_idx, root);
		}

		int ik_a_bone_idx = editing_skeleton->find_bone(ik_tips[i]);
		int ik_b_bone_idx = editing_skeleton->get_bone_parent(ik_a_bone_idx);
		int ik_c_bone_idx = editing_skeleton->get_bone_parent(ik_b_bone_idx);
		{
			// Set the rest position for the target bone
			editing_skeleton->set_bone_rest(target_bone_idx, root_rest.affine_inverse() * editing_skeleton->get_bone_global_rest(ik_a_bone_idx));
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
			editing_skeleton->set_bone_rest(magnet_bone_idx, root_rest.affine_inverse() * magnet_trf);
			editing_skeleton->reset_bone_pose(magnet_bone_idx);
		}

		// Create the constraint
		Ref<EPASAnimationEditorIKJoint> ik_constraint;
		ik_constraint.instantiate();

		Ref<FABRIKSolver> fabrik_solver;
		fabrik_solver.instantiate();
		fabrik_solver->set_joint_count(3);
		/*fabrik_solver->set_joint_transform(0, editing_skeleton->get_bone_global_rest(ik_a_bone_idx));
		fabrik_solver->set_joint_transform(1, editing_skeleton->get_bone_rest(ik_b_bone_idx));
		fabrik_solver->set_joint_transform(2, editing_skeleton->get_bone_rest(ik_c_bone_idx));
		fabrik_solver->calculate_distances();*/

		fabrik_solver->set_use_pole_constraint(true);
		fabrik_solver->set_joint_hinge_enabled(1, true);
		fabrik_solver->set_joint_rotation_limit_enabled(1, true);
		fabrik_solver->set_joint_rotation_limit_min(1, Vector3(Math::deg_to_rad(10.0f), Math::deg_to_rad(-180.0f), Math::deg_to_rad(-180.0f)));
		fabrik_solver->set_joint_rotation_limit_max(1, Vector3(Math::deg_to_rad(180.0f), Math::deg_to_rad(180.0f), Math::deg_to_rad(180.0f)));

		ik_constraint->set_fabrik_solver(fabrik_solver);

		ik_constraint->set_ik_magnet_bone_name(ik_magnet_bone_name);
		ik_constraint->set_ik_target_bone_name(ik_target_bone_name);
		ik_constraint->set_ik_tip_bone_name(ik_tips[i]);
		ik_constraint->set_use_magnet(false);

		// Create selection handles
		Ref<Selection> ik_target_handle;
		ik_target_handle.instantiate();
		Ref<Selection> ik_magnet_handle;
		ik_magnet_handle.instantiate();

		ik_target_handle->bone_idx = target_bone_idx;
		ik_target_handle->ik_joint = ik_constraint;
		ik_target_handle->type = Selection::SelectionType::IK_HANDLE;
		ik_target_handle->group = Selection::SelectionGroup::IK_GROUP;

		ik_magnet_handle->bone_idx = magnet_bone_idx;
		ik_magnet_handle->is_ik_magnet = true;
		ik_magnet_handle->ik_joint = ik_constraint;
		ik_magnet_handle->type = Selection::SelectionType::IK_HANDLE;
		ik_magnet_handle->group = Selection::SelectionGroup::IK_GROUP;

		selection_handles.push_back(ik_target_handle);
		selection_handles.push_back(ik_magnet_handle);
		ik_constraints.push_back(ik_constraint);
	}
}

void EPASAnimationEditor::_apply_constraints(const Ref<EPASPose> &p_pose) {
	if (!editing_skeleton) {
		return;
	}
	for (int i = 0; i < ik_constraints.size(); i++) {
		Ref<EPASAnimationEditorIKJoint> ik_constraint = ik_constraints[i];

		int target_bone_idx = editing_skeleton->find_bone(ik_constraint->get_ik_target_bone_name());
		int magnet_bone_idx = editing_skeleton->find_bone(ik_constraint->get_ik_magnet_bone_name());

		int c_bone_idx = editing_skeleton->find_bone(ik_constraint->get_ik_tip_bone_name());
		int b_bone_idx = editing_skeleton->get_bone_parent(c_bone_idx);
		int a_bone_idx = editing_skeleton->get_bone_parent(b_bone_idx);

		ERR_FAIL_COND_MSG(c_bone_idx == -1 || b_bone_idx == -1 || a_bone_idx == -1, "Invalid IK constraint");

		Transform3D a_bone_trf = editing_skeleton->get_bone_global_pose(a_bone_idx);
		Transform3D b_bone_trf = editing_skeleton->get_bone_pose(b_bone_idx);
		Transform3D c_bone_trf = editing_skeleton->get_bone_pose(c_bone_idx);
		Transform3D target_bone_trf = editing_skeleton->get_bone_global_pose(target_bone_idx);
		Vector3 magnet_target_pos = editing_skeleton->get_bone_global_pose(magnet_bone_idx).origin;

		Ref<FABRIKSolver> fabrik_solver = ik_constraint->get_fabrik_solver();
		fabrik_solver->set_joint_transform(0, a_bone_trf);
		fabrik_solver->set_joint_transform(1, b_bone_trf);
		fabrik_solver->set_joint_transform(2, c_bone_trf);
		fabrik_solver->calculate_distances();
		fabrik_solver->set_target_position(target_bone_trf.origin);
		fabrik_solver->set_pole_position(magnet_target_pos);

		fabrik_solver->solve(10);

		String a_bone_name = editing_skeleton->get_bone_name(a_bone_idx);
		String b_bone_name = editing_skeleton->get_bone_name(b_bone_idx);
		String c_bone_name = ik_constraint->get_ik_tip_bone_name();

		int a_parent_transform_idx = editing_skeleton->get_bone_parent(a_bone_idx);

		ERR_FAIL_COND(a_parent_transform_idx == -1);

		Transform3D a_parent_global_transform = editing_skeleton->get_bone_global_pose(a_parent_transform_idx);
		Quaternion a_bone_result_local = (a_parent_global_transform.affine_inverse() * fabrik_solver->get_joint_transform(0)).basis.get_rotation_quaternion();
		Quaternion b_bone_result_local = fabrik_solver->get_joint_transform(1).basis.get_rotation_quaternion();

		const int bone_indices[] = { a_bone_idx, b_bone_idx };
		const Quaternion *bone_rots[] = { &a_bone_result_local, &b_bone_result_local };

		for (int j = 0; j < 2; j++) {
			String bone_name = editing_skeleton->get_bone_name(bone_indices[j]);
			if (!p_pose->has_bone(bone_name)) {
				p_pose->create_bone(bone_name);
			}
			p_pose->set_bone_rotation(bone_name, *bone_rots[j]);
		}

		if (!p_pose->has_bone(c_bone_name)) {
			p_pose->create_bone(c_bone_name);
		}
		// Align the tip of rotation
		Transform3D b_global_trf = p_pose->calculate_bone_global_transform(b_bone_name, editing_skeleton, epas_controller->get_base_pose());
		Quaternion c_rot = b_global_trf.basis.get_rotation_quaternion().inverse() * target_bone_trf.basis.get_rotation_quaternion();
		p_pose->set_bone_rotation(c_bone_name, c_rot);
	}
}

void EPASAnimationEditor::_apply_constraints_to_current_frame() {
	Ref<EPASPose> current_pose = get_current_pose();
	if (!current_pose.is_valid()) {
		return;
	}
	_apply_constraints(current_pose);
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
	current_animation->get_editor_animation()->add_keyframe(p_keyframe);
}

void EPASAnimationEditor::_remove_keyframe(Ref<EPASKeyframe> p_keyframe) {
	for (int i = 0; i < keyframe_cache.size(); i++) {
		if (keyframe_cache[i]->keyframe == p_keyframe) {
			memdelete(keyframe_cache[i]);
			keyframe_cache.remove_at(i);
			break;
		}
	}
	current_animation->get_editor_animation()->erase_keyframe(p_keyframe);
}

void EPASAnimationEditor::_rebuild_keyframe_cache() {
	for (int i = 0; i < keyframe_cache.size(); i++) {
		memdelete(keyframe_cache[i]);
	}
	keyframe_cache.clear();

	ERR_FAIL_COND(!current_animation->get_editor_animation().is_valid());

	keyframe_cache.resize(current_animation->get_editor_animation()->get_keyframe_count());
	for (int i = 0; i < current_animation->get_editor_animation()->get_keyframe_count(); i++) {
		AnimationKeyframeCache *kc = memnew(AnimationKeyframeCache(current_animation->get_editor_animation()->get_keyframe(i)));
		keyframe_cache.set(i, kc);
	}
}

void EPASAnimationEditor::_draw_ui() {
	ImGuiIO io = ImGui::GetIO();
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("New")) {
				if (current_model) {
					current_model->queue_free();
					current_model = nullptr;
					editing_skeleton = nullptr;
				}
				for (int i = 0; i < keyframe_cache.size(); i++) {
					memdelete(keyframe_cache[i]);
				}
				keyframe_cache.clear();
				undo_redo->clear_history();
			}
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
			if (ImGui::MenuItem("Load placeholder scene")) {
				placeholder_load_dialog->popup_centered_ratio(0.75);
			}
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Editor")) {
			if (ImGui::MenuItem("Copy pose")) {
				ui_info.pose_copy_buffer = epas_controller->get_output_pose()->duplicate();
			}
			if (ImGui::MenuItem("Paste pose", nullptr, false, ui_info.pose_copy_buffer.is_valid())) {
				if (ui_info.pose_copy_buffer.is_valid()) {
					AnimationKeyframeCache *kf = get_keyframe(current_frame);
					if (kf) {
						kf->keyframe->set_pose(ui_info.pose_copy_buffer->duplicate());
					}
				}
			}
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("View")) {
			if (ImGui::MenuItem("ConstraintDBG")) {
				ui_info.constraintdbg_window_visible = !ui_info.constraintdbg_window_visible;
			}
			if (ImGui::MenuItem("PoseDBG")) {
				ui_info.posedbg_window_visible = !ui_info.posedbg_window_visible;
			}
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Tools")) {
			if (ImGui::MenuItem("Event editor", nullptr, events_editor->open)) {
				events_editor->open = !events_editor->open;
			}
			ImGui::Separator();
			if (ImGui::MenuItem("Run test map")) {
				Ref<PackedScene> test_map = ResourceLoader::load("res://test2.tscn");
				if (test_map.is_valid()) {
					get_viewport()->set_embedding_subwindows(false);
					Window *new_window = memnew(Window);
					new_window->set_use_own_world_3d(true);
					new_window->add_child(test_map->instantiate());
					new_window->set_size(Vector2i(1280, 720));
					add_child(new_window);
				}
			}
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
			if (ImGui::MenuItem("Copy FK to IK")) {
				_copy_fk_to_ik();
			}
			if (ImGui::MenuItem("Root motion post-process funky")) {
				Vector<StringName> children_of_root;
				Ref<EPASPose> base_pose = epas_controller->get_base_pose();
				{
					Vector<int> bone_children = editing_skeleton->get_bone_children(editing_skeleton->find_bone("root"));
					for (int i = 0; i < bone_children.size(); i++) {
						children_of_root.push_back(editing_skeleton->get_bone_name(bone_children[i]));
					}
				}
				Vector3 root_offset = Vector3();
				for (int i = 0; i < current_animation->get_editor_animation()->get_keyframe_count(); i++) {
					Ref<EPASKeyframe> kf = current_animation->get_editor_animation()->get_keyframe(i);
					Ref<EPASPose> pose = kf->get_pose();

					HashMap<StringName, Transform3D> children_of_root_global_trfs;

					for (StringName sn : children_of_root) {
						if (!pose->has_bone(sn)) {
							pose->create_bone(sn);
						}
						children_of_root_global_trfs.insert(sn, pose->calculate_bone_global_transform(sn, editing_skeleton, base_pose));
						print_line(sn);
					}

					Vector3 new_root_pos = pose->calculate_bone_global_transform("spine", editing_skeleton, base_pose).origin;
					new_root_pos.y = 0.0f;
					Transform3D new_root_trf(Basis(), new_root_pos);

					if (!pose->has_bone("root")) {
						pose->create_bone("root");
					}

					pose->set_bone_position("root", new_root_trf.origin);
					pose->set_bone_rotation("root", new_root_trf.basis.get_rotation_quaternion());

					for (KeyValue<StringName, Transform3D> kv : children_of_root_global_trfs) {
						const Transform3D new_bone_trf = new_root_trf.affine_inverse() * kv.value;
						pose->set_bone_position(kv.key, new_bone_trf.origin);
						pose->set_bone_rotation(kv.key, new_bone_trf.basis.get_rotation_quaternion());
					}
					if (i == 0) {
						root_offset = -new_root_trf.origin;
					}
					pose->set_bone_position("root", new_root_trf.origin + root_offset);
				}
			}
			if (ImGui::MenuItem("Convert legacy IK")) {
				// In the past IK bones used to be children of nobody, nowadays they are children of root
				if (editing_skeleton && current_animation->get_meta("_has_old_ik", true)) {
					for (int i = 0; i < current_animation->get_keyframe_count(); i++) {
						Ref<EPASKeyframe> kf = current_animation->get_keyframe(i);
						Ref<EPASPose> pose = kf->get_pose();
						for (int j = 0; j < ik_constraints.size(); j++) {
							const Ref<EPASAnimationEditorIKJoint> ik_constraint = ik_constraints[j];
							StringName target_bone_name = ik_constraint->get_ik_target_bone_name();
							StringName magnet_bone_name = ik_constraint->get_ik_magnet_bone_name();
							StringName bones[2] = { target_bone_name, magnet_bone_name };
							Transform3D root_trf = pose->get_bone_transform("root", epas_controller->get_base_pose());
							for (int k = 0; k < 2; k++) {
								StringName bone_name = bones[k];
								if (pose->has_bone(bone_name)) {
									Transform3D curr_bone_global = pose->get_bone_transform(bone_name, epas_controller->get_base_pose());
									Transform3D new_trf = root_trf.affine_inverse() * curr_bone_global;
									pose->set_bone_position(bone_name, new_trf.origin);
									if (bone_name != magnet_bone_name) {
										pose->set_bone_rotation(bone_name, new_trf.basis.get_rotation_quaternion());
									}
								}
							}
						}
					}
				}
			}
			if (ImGui::MenuItem("RootMover™", nullptr, ui_info.root_mover_window_visible)) {
				ui_info.root_mover_window_visible = !ui_info.root_mover_window_visible;
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
			ImGui::Combo("##Guizmo mode combo", (int *)&guizmo_mode, "Local\0Global\0\0");
			ImGui::SameLine();
			if (ImGui::Button("GroupVis")) {
				ImGui::OpenPopup("GroupVisPopup");
			}

			if (ImGui::BeginPopup("GroupVisPopup")) {
				const char *GROUP_NAMES[] = {
					"FK Center",
					"FK Left",
					"FK Right",
					"FK Fingers",
					"IK Bones",
					"Warp Points",
				};
				String eye_icon = String::utf8(FONT_REMIX_ICON_EYE_LINE " ");
				String eye_off_icon = String::utf8(FONT_REMIX_ICON_EYE_CLOSE_LINE " ");
				ImGui::PushItemFlag(ImGuiItemFlags_SelectableDontClosePopup, true);
				for (int i = 0; i < ui_info.group_visibility.size(); i++) {
					String name = ui_info.group_visibility[i] ? eye_icon : eye_off_icon;
					name += String(GROUP_NAMES[i]);
					if (ImGui::Selectable(name.utf8().get_data())) {
						ui_info.group_visibility.set(i, !ui_info.group_visibility[i]);
					}
				}
				ImGui::PopItemFlag();
				ImGui::EndPopup();
			}

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
			ImGui::Checkbox("Enabled RTIK", &ui_info.enable_rt_ik);
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
		ImGui::DockBuilderDockWindow("Inspector", dock_up_r);
		ImGui::DockBuilderDockWindow("Settings", dock_up_r);
		ImGui::DockBuilderDockWindow("Undo History", dock_up_r);
		ImGui::DockBuilderDockWindow("Warp Points", dock_up_r);
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
			if (!current_pose.is_valid()) {
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
				epas_animation_node->set_looping_enabled(false);
			}
		} else if (ImGui::Button(FONT_REMIX_ICON_PLAY_LINE)) {
			epas_animation_node->set_playback_mode(EPASAnimationNode::AUTOMATIC);
			epas_animation_node->set_looping_enabled(true);
		}
		ImGui::SameLine();
		ImGui::Text("%d/%d", current_frame, end_frame);

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
					undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));
					undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));
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

	if (ImGui::Begin("Inspector")) {
		if (editing_selection_handles.size() > 0 && get_current_pose().is_valid()) {
			const Ref<Selection> selection_handle = selection_handles[editing_selection_handles[0]];
			if (selection_handle.is_valid()) {
				Transform3D handle_trf;
				HBTransformConversions::mat_to_trf(current_handle_trf_matrix.ptr(), handle_trf);
				float *origin = current_handle_trf_matrix.ptrw() + 12;
				bool pasted_pos = false;

				if (ImGui::Button("Copy##pos")) {
					ui_info.copy_buffer = handle_trf.origin;
				}
				ImGui::SameLine();
				if (ImGui::Button("Paste##pos")) {
					pasted_pos = true;
					origin[0] = ui_info.copy_buffer.x;
					origin[1] = ui_info.copy_buffer.y;
					origin[2] = ui_info.copy_buffer.z;
				}

				if (ImGui::InputFloat3("Position", origin, "%.6f") || pasted_pos) {
					_apply_handle_transform(ImGuizmo::OPERATION::TRANSLATE);
				}
				Vector3 euler = handle_trf.get_basis().get_rotation_quaternion().get_euler();
				euler.x = Math::rad_to_deg(euler.x);
				euler.y = Math::rad_to_deg(euler.y);
				euler.z = Math::rad_to_deg(euler.z);
				bool pasted_rot = false;
				if (ImGui::Button("Copy##rot")) {
					ui_info.copy_buffer = euler;
				}
				ImGui::SameLine();
				if (ImGui::Button("Paste##rot")) {
					euler = ui_info.copy_buffer;
					pasted_rot = true;
				}

				if (ImGui::InputFloat3("Rotation", euler.coord) || pasted_rot) {
					euler.x = Math::deg_to_rad(euler.x);
					euler.y = Math::deg_to_rad(euler.y);
					euler.z = Math::deg_to_rad(euler.z);
					handle_trf.set_basis(Basis::from_euler(euler));
					HBTransformConversions::trf_to_mat(handle_trf, current_handle_trf_matrix.ptrw());
					_apply_handle_transform(ImGuizmo::ROTATE);
				}
				ImGui::Text("COPYBUFF %.2f, %.2f, %.2f", ui_info.copy_buffer.x, ui_info.copy_buffer.y, ui_info.copy_buffer.z);
			}
		}
	}
	ImGui::End();

	if (ImGui::Begin("Settings")) {
		EPASAnimation::InterpolationMethod interp_method = current_animation->get_editor_interpolation_method();
		EPASAnimation::InterpolationMethod original_interp_method = interp_method;
		const char *items[] = { "Step", "Linear", "Bicubic", "Bicublic Clamped" };
		ImGui::SetNextItemWidth(80);
		ImGui::Combo("Interpolation", reinterpret_cast<int *>(&interp_method), items, IM_ARRAYSIZE(items));
		if (interp_method != original_interp_method) {
			epas_animation_node->set_interpolation_method(interp_method);
			current_animation->set_editor_interpolation_method(interp_method);
		}
		ImGui::InputInt("Length", &end_frame);
	}
	ImGui::End();

	Ref<EPASPose> current_pose = get_current_pose();
	if (ui_info.posedbg_window_visible) {
		if (ImGui::Begin("PoseDBG", &ui_info.posedbg_window_visible)) {
			if (current_pose.is_valid()) {
				EPASAnimationEditor::AnimationKeyframeCache *f = get_keyframe(current_frame);
				ImGui::Text("%d %d", f->frame_time, f->temporary_frame_time);
				ImGui::Text("%f", f->keyframe->get_time() * 60.0f);
				for (int i = 0; i < current_pose->get_bone_count(); i++) {
					StringName bone_name = current_pose->get_bone_name(i);
					String bone_name_str = bone_name;
					if (ImGui::CollapsingHeader(bone_name_str.utf8().get_data())) {
						ImGui::PushID(bone_name.data_unique_pointer());
						if (current_pose->get_bone_has_position(bone_name)) {
							ImGui::AlignTextToFramePadding();
							ImGui::TextUnformatted(vformat("Pos %s", current_pose->get_bone_position(bone_name)).utf8().get_data());
							ImGui::SameLine();
							if (ImGui::Button("X##pos")) {
								undo_redo->create_action("Remove position from keyframe");
								undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_position).bind(bone_name, false));
								undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_position).bind(bone_name, true));
								undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));
								undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));
								undo_redo->commit_action();
							}
						}
						if (current_pose->get_bone_has_rotation(bone_name)) {
							ImGui::AlignTextToFramePadding();
							ImGui::TextUnformatted(vformat("Rot %s", current_pose->get_bone_rotation(bone_name).get_euler(EulerOrder::XYZ) * 180.0f / Math_PI).utf8().ptr());
							ImGui::SameLine();
							if (ImGui::Button("X##rot")) {
								undo_redo->create_action("Remove rotation from keyframe");
								undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_rotation).bind(bone_name, false));
								undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_rotation).bind(bone_name, true));
								undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));
								undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));
								undo_redo->commit_action();
							}
						}
						if (current_pose->get_bone_has_scale(bone_name)) {
							ImGui::AlignTextToFramePadding();
							ImGui::TextUnformatted(vformat("Scale %s", current_pose->get_bone_scale(bone_name)).utf8().ptr());
							ImGui::SameLine();
							if (ImGui::Button("X##scale")) {
								undo_redo->create_action("Remove scale from keyframe");
								undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_scale).bind(bone_name, false));
								undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_has_scale).bind(bone_name, true));
								undo_redo->commit_action();
							}
						}
						ImGui::PopID();
					}
				}
			}
		}
		ImGui::End();
	}

	if (ui_info.constraintdbg_window_visible) {
		if (ImGui::Begin("ConstraintDBG", &ui_info.constraintdbg_window_visible)) {
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
	}
	if (ImGui::Begin("Undo History")) {
		ImGui::Text("Version: %lu", undo_redo->get_version());
		ImGui::PushItemWidth(-1);
		if (ImGui::BeginListBox("##Actions")) {
			int current_action = undo_redo->get_current_action();
			for (int i = 0; i < undo_redo->get_history_count(); i++) {
				ImGui::Selectable(undo_redo->get_action_name(i).utf8().get_data(), i == current_action);
			}
			ImGui::EndListBox();
		}
	}
	ImGui::End();
	if (ImGui::Begin("Warp Points")) {
		static char input[128] = {};
		ImGui::InputText("##name", input, 128);
		ImGui::SameLine();
		if (ImGui::Button(FONT_REMIX_ICON_ADD)) {
			String wp_name = String(input);
			if (!wp_name.is_empty()) {
				if (current_animation->has_warp_point(wp_name)) {
					_show_error("Animation already has a warp point with that name!");
				} else {
					Ref<EPASWarpPoint> warp_point;
					warp_point.instantiate();
					warp_point->set_point_name(wp_name);
					undo_redo->create_action("Add new warp point");
					undo_redo->add_do_method(callable_mp((EPASAnimation *)current_animation.ptr(), &EPASAnimation::add_warp_point).bind(warp_point));
					undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_add_warp_point).bind(warp_point));
					undo_redo->add_undo_method(callable_mp((EPASAnimation *)current_animation.ptr(), &EPASAnimation::erase_warp_point).bind(warp_point));
					undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_remove_warp_point).bind(warp_point));
					undo_redo->commit_action();
				}
			}
		}
		if (current_animation->get_warp_point_count() > 0) {
			int selected_warp_point_idx = CLAMP(ui_info.selected_warp_point, 0, current_animation->get_warp_point_count());
			const char *preview_value = String(current_animation->get_warp_point(selected_warp_point_idx)->get_point_name()).utf8().get_data();
			if (ImGui::BeginCombo("##warp points", preview_value)) {
				for (int i = 0; i < current_animation->get_warp_point_count(); i++) {
					Ref<EPASWarpPoint> wp = current_animation->get_warp_point(i);
					bool selected = i == selected_warp_point_idx;
					if (ImGui::Selectable(String(wp->get_point_name()).utf8().get_data())) {
						ui_info.selected_warp_point = i;
					}
					if (selected) {
						ImGui::SetItemDefaultFocus();
					}
				}
				ImGui::EndCombo();
			}
			selected_warp_point_idx = CLAMP(ui_info.selected_warp_point, 0, current_animation->get_warp_point_count());

			// Warp point delete button
			ImGui::SameLine();
			ImGui::BeginDisabled(selected_warp_point_idx == -1);
			if (ImGui::Button(FONT_REMIX_ICON_DELETE_BIN_LINE) && selected_warp_point_idx != -1) {
				Ref<EPASWarpPoint> wp = current_animation->get_warp_point(selected_warp_point_idx);
				undo_redo->create_action("Remove selected warp point");
				undo_redo->add_do_method(callable_mp((EPASAnimation *)current_animation.ptr(), &EPASAnimation::erase_warp_point).bind(wp));
				undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_remove_warp_point).bind(wp));
				undo_redo->add_undo_method(callable_mp((EPASAnimation *)current_animation.ptr(), &EPASAnimation::add_warp_point).bind(wp));
				undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_add_warp_point).bind(wp));
				undo_redo->commit_action();
				selected_warp_point_idx = -1;
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Remove selected warp point");
			}
			ImGui::EndDisabled();

			// Warp point properties
			if (selected_warp_point_idx != -1) {
				Ref<EPASWarpPoint> wp = current_animation->get_warp_point(selected_warp_point_idx);
				const char *WARP_PROPS[] = {
					"facing_start", "facing_end",
					"rotation_start", "rotation_end",
					"translation_start", "translation_end"
				};
				for (int i = 0; i < 6; i += 2) {
					int warp_prop_vals[2] = { (int)wp->get(WARP_PROPS[i]), (int)wp->get(WARP_PROPS[i + 1]) };
					String name = String(WARP_PROPS[i]).capitalize().split(" ")[0];
					ImGui::PushItemWidth(-100);
					if (ImGui::InputInt2(name.utf8().get_data(), warp_prop_vals)) {
						undo_redo->create_action(vformat("Set warp point %s", name), UndoRedo::MergeMode::MERGE_ENDS);
						String start_prop_name = WARP_PROPS[i];
						String end_prop_name = WARP_PROPS[i + 1];
						undo_redo->add_undo_property(wp.ptr(), start_prop_name, wp->get(start_prop_name));
						undo_redo->add_do_property(wp.ptr(), start_prop_name, warp_prop_vals[0]);
						undo_redo->add_undo_property(wp.ptr(), end_prop_name, wp->get(end_prop_name));
						undo_redo->add_do_property(wp.ptr(), end_prop_name, warp_prop_vals[1]);
						undo_redo->commit_action();
					}
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
		HBTransformConversions::trf_to_mat(view_trf, view_matrix.ptrw());

		Projection proj;
		float aspect = io.DisplaySize.x / io.DisplaySize.y;
		proj.set_perspective(cam->get_fov(), aspect, cam->get_near(), cam->get_far(), cam->get_keep_aspect_mode() == Camera3D::KEEP_WIDTH);
		HBTransformConversions::proj_to_mat(proj, projection_matrix.ptrw());

		ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
		if (editing_selection_handles.size() > 0) {
			const Ref<Selection> selection_handle = selection_handles[editing_selection_handles[0]];
			PackedFloat32Array trf_delta = current_handle_trf_matrix.duplicate();
			Transform3D trf_original;
			HBTransformConversions::mat_to_trf(current_handle_trf_matrix.ptr(), trf_original);
			manipulated = ImGuizmo::Manipulate(view_matrix.ptr(), projection_matrix.ptr(), guizmo_operation, guizmo_mode, current_handle_trf_matrix.ptrw(), trf_delta.ptrw());
			if (manipulated || ImGuizmo::IsUsing()) {
				_apply_handle_transform(guizmo_operation, trf_delta);
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

	if (ui_info.root_mover_window_visible) {
		if (ImGui::Begin("RootMover(tm)", &ui_info.root_mover_window_visible)) {
			static float root_mover_target[3] = { 0.0f };
			current_pose = get_current_pose();
			ImGui::InputFloat3("New position", root_mover_target);
			ImGui::BeginDisabled(!current_pose.is_valid());
			if (ImGui::Button("Apply")) {
				Vector3 old_root_pos = current_pose->get_bone_position("root", epas_controller->get_base_pose());
				Vector3 new_root_pos = Vector3(root_mover_target[0], root_mover_target[1], root_mover_target[2]);
				Vector3 diff = new_root_pos - old_root_pos;

				Vector<int> children = editing_skeleton->get_bone_children(editing_skeleton->find_bone("root"));
				undo_redo->create_action("RootMove");
				for (int i = 0; i < children.size(); i++) {
					StringName bone_name = editing_skeleton->get_bone_name(children[i]);
					if (current_pose->has_bone(bone_name)) {
						Vector3 curr_pos = current_pose->get_bone_position(bone_name, epas_controller->get_base_pose());
						undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind(bone_name, curr_pos - diff));
						undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind(bone_name, curr_pos));
					}
				}
				undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind("root", new_root_pos));
				undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind("root", old_root_pos));

				undo_redo->commit_action();
			}
			ImGui::EndDisabled();
		}
		ImGui::End();
	}
}

void EPASAnimationEditor::_apply_handle_transform(ImGuizmo::OPERATION p_operation, PackedFloat32Array p_delta) {
	// Commit current imguizmo transform to undo_redo
	if (editing_selection_handles.size() == 0) {
		return;
	}
	bool created_action = false;
	Ref<EPASPose> current_pose = get_current_pose();
	ERR_FAIL_COND(!current_pose.is_valid());
	// Apply the handle transform based on the user's input
	for (int handle_i : editing_selection_handles) {
		Ref<Selection> selection_handle = selection_handles[handle_i];
		// Handle bone trf
		if (selection_handle->type == Selection::SelectionType::IK_HANDLE || selection_handle->type == Selection::SelectionType::FK_BONE) {
			// No need to get fancy since both fk_bone_idx and ik_bone_idx are ints
			int editing_bone = selection_handle->bone_idx;
			StringName bone_name = editing_skeleton->get_bone_name(editing_bone);
			Transform3D new_bone_trf;
			Transform3D old_bone_trf = editing_skeleton->get_bone_pose(editing_bone);
			_world_to_bone_trf(editing_bone, current_handle_trf_matrix.ptr(), new_bone_trf);
			if (p_delta.size() == 16) {
				Transform3D trf_delta;
				HBTransformConversions::mat_to_trf(p_delta.ptr(), trf_delta);
				Transform3D old_bone_global = editing_skeleton->get_global_transform() * editing_skeleton->get_bone_global_pose(editing_bone);
				new_bone_trf = trf_delta * old_bone_global;
				PackedFloat32Array r;
				r.resize(16);
				HBTransformConversions::trf_to_mat(new_bone_trf, r.ptrw());
				_world_to_bone_trf(editing_bone, r.ptr(), new_bone_trf);
			}

			if (!current_pose->has_bone(bone_name)) {
				current_pose->create_bone(bone_name);
			}
			String action_template = "%s bone %s";
			switch (p_operation) {
				case ImGuizmo::TRANSLATE: {
					if (!created_action) {
						undo_redo->create_action(vformat(action_template, "Translate", bone_name), UndoRedo::MERGE_ENDS);
					}
					created_action = true;
					undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind(bone_name, new_bone_trf.origin));
					undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_position).bind(bone_name, old_bone_trf.origin));
				} break;
				case ImGuizmo::ROTATE: {
					if (!created_action) {
						undo_redo->create_action(vformat(action_template, "Rotate", bone_name), UndoRedo::MERGE_ENDS);
					}
					created_action = true;
					undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_rotation).bind(bone_name, new_bone_trf.basis.get_rotation_quaternion()));
					undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_rotation).bind(bone_name, old_bone_trf.basis.get_rotation_quaternion()));

				} break;
				case ImGuizmo::SCALE: {
					created_action = true;
					undo_redo->create_action(vformat(action_template, "Scale", bone_name), UndoRedo::MERGE_ENDS);
					undo_redo->add_do_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_scale).bind(bone_name, new_bone_trf.get_basis().get_scale()));
					undo_redo->add_undo_method(callable_mp(current_pose.ptr(), &EPASPose::set_bone_scale).bind(bone_name, old_bone_trf.basis.get_scale()));
				} break;
				default: {
				};
			}
		} else if (selection_handle->type == Selection::SelectionType::WARP_POINT) {
			created_action = true;
			Transform3D new_trf;
			Transform3D prev_trf;
			HBTransformConversions::mat_to_trf(current_handle_trf_matrix.ptr(), new_trf);
			HBTransformConversions::mat_to_trf(prev_handle_trf_matrix.ptr(), prev_trf);
			undo_redo->create_action("Change warp point transform", UndoRedo::MERGE_ENDS);
			Ref<EPASWarpPoint> warp_point = selection_handle->warp_point;
			undo_redo->add_do_method(callable_mp(warp_point.ptr(), &EPASWarpPoint::set_transform).bind(new_trf));
			undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::queue_warp_point_sphere_update));
			undo_redo->add_undo_method(callable_mp(warp_point.ptr(), &EPASWarpPoint::set_transform).bind(prev_trf));
			undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::queue_warp_point_sphere_update));
		}
	}

	if (!current_pose.is_valid()) {
		return;
	}

	if (created_action) {
		undo_redo->add_do_method(callable_mp(epas_controller, &EPASController::advance).bind(0.0f));
		undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));

		undo_redo->add_undo_method(callable_mp(epas_controller, &EPASController::advance).bind(0.0f));
		undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_apply_constraints_to_current_frame));

		undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_update_editing_handle_trf));
		undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_update_editing_handle_trf));

		undo_redo->commit_action();
	}
}

bool EPASAnimationEditor::_is_playing() {
	return epas_animation_node->get_playback_mode() == EPASAnimationNode::AUTOMATIC;
}

void EPASAnimationEditor::_show_error(const String &error) {
	current_error = error;
}

void EPASAnimationEditor::_add_warp_point(Ref<EPASWarpPoint> p_warp_point) {
	ERR_FAIL_COND(!p_warp_point.is_valid());
	Ref<Selection> handle;
	handle.instantiate();
	handle->warp_point = p_warp_point;
	handle->type = Selection::SelectionType::WARP_POINT;
	handle->group = Selection::SelectionGroup::WARP_POINT_GROUP;
	selection_handles.push_back(handle);
	queue_warp_point_sphere_update();
}

void EPASAnimationEditor::_remove_warp_point(Ref<EPASWarpPoint> p_warp_point) {
	for (int i = 0; i < selection_handles.size(); i++) {
		if (selection_handles[i]->type == Selection::WARP_POINT) {
			if (selection_handles[i]->warp_point == p_warp_point) {
				selection_handles.remove_at(i);
				break;
			}
		}
	}
	ui_info.selected_warp_point = -1;
	queue_warp_point_sphere_update();
}

void EPASAnimationEditor::queue_warp_point_sphere_update() {
	warp_point_sphere_update_queued = true;
}

void EPASAnimationEditor::_load_placeholder_scene(const String &p_path) {
	Ref<PackedScene> ps = ResourceLoader::load(p_path);
	if (ps.is_valid()) {
		Node *node_c = ps->instantiate();
		if (!node_c) {
			return;
		}
		Node3D *node = Object::cast_to<Node3D>(node_c);
		if (node) {
			add_child(node);
			placeholder_scene = node;
		} else {
			_show_error("Scene was not a 3D scene, are you sure you are doing this right?");
			node_c->queue_free();
		}
	}
}

void EPASAnimationEditor::_copy_fk_to_ik() {
	StringName ik_tips[] = {
		"hand.L",
		"hand.R",
		"foot.L",
		"foot.R"
	};
	StringName fk_magnet_bones[] = {
		"forearm.L",
		"forearm.R",
		"shin.L",
		"shin.R"
	};

	StringName fk_bones[] = {
		"hand.L",
		"hand.R",
		"foot.L",
		"foot.R"
	};

	Ref<EPASPose> base_pose = epas_controller->get_base_pose();
	for (int i = 0; i < current_animation->get_editor_animation()->get_keyframe_count(); i++) {
		Ref<EPASKeyframe> kf = current_animation->get_editor_animation()->get_keyframe(i);
		Ref<EPASPose> pose = kf->get_pose();
		const Transform3D root_trf = pose->get_bone_transform("root", base_pose);
		for (int j = 0; j < static_cast<int>(std::size(ik_tips)); j++) {
			String ik_magnet_bone_name = "IK.Magnet." + ik_tips[j];
			String ik_target_bone_name = "IK." + ik_tips[j];
			if (!pose->has_bone(ik_magnet_bone_name)) {
				pose->create_bone(ik_magnet_bone_name);
			}
			if (!pose->has_bone(ik_target_bone_name)) {
				pose->create_bone(ik_target_bone_name);
			}

			Transform3D magnet_trf = pose->calculate_bone_global_transform(fk_magnet_bones[j], editing_skeleton, base_pose);
			Transform3D target_trf = pose->calculate_bone_global_transform(fk_bones[j], editing_skeleton, base_pose);

			pose->set_bone_position(ik_magnet_bone_name, root_trf.xform_inv(magnet_trf.origin));
			pose->set_bone_position(ik_target_bone_name, root_trf.xform_inv(target_trf.origin));
			pose->set_bone_rotation(ik_target_bone_name, root_trf.basis.inverse() * target_trf.basis.get_rotation_quaternion());
		}
	}
}

void EPASAnimationEditor::open_file(const String &p_path) {
	Error err;
	Ref<EPASEditorAnimation> animation = ResourceLoader::load(p_path, "EPASEditorAnimation", ResourceFormatLoader::CACHE_MODE_REUSE, &err);
	if (err == OK) {
		if (animation.is_valid()) {
			undo_redo->clear_history();
			if (!animation->get_editor_animation().is_valid()) {
				// Copy all keyframes over
				Ref<EPASAnimation> editor_animation;
				editor_animation.instantiate();
				for (int i = 0; i < animation->get_keyframe_count(); i++) {
					Ref<EPASKeyframe> og_kf = animation->get_keyframe(i);
					Ref<EPASKeyframe> kf;
					kf.instantiate();
					kf->set_time(og_kf->get_time());
					kf->set_pose(og_kf->get_pose()->duplicate());
					editor_animation->add_keyframe(kf);
				}
				animation->set_editor_animation(editor_animation);
			}
			animation->clear_keyframes();
			epas_animation_node->set_interpolation_method(animation->get_editor_interpolation_method());
			if (animation->has_meta("__editor_model_path")) {
				String model_path = animation->get_meta("__editor_model_path");
				load_model(model_path);
			}
			set_animation(animation);
			if (animation->get_meta("__editor_humanoid_ik", false)) {
				_create_eirteam_humanoid_ik();
			}
			if (animation->has_meta("__editor_group_visibility")) {
				Array group_vis = animation->get_meta("__editor_group_visibility");
				if (group_vis.size() <= ui_info.group_visibility.size()) {
					for (int i = 0; i < group_vis.size(); i++) {
						ui_info.group_visibility.set(i, group_vis[i]);
					}
				}
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
	epas_controller->set_playback_process_mode(EPASController::MANUAL);
	editing_skeleton = skel;
	current_model = node_3d;

	// Add selection handles
	selection_handles.clear();
	for (int i = 0; i < skel->get_bone_count(); i++) {
		Ref<Selection> selection_handle;
		String bone_name = skel->get_bone_name(i);
		selection_handle.instantiate();
		selection_handle->bone_idx = i;
		if (bone_name.begins_with("palm") || bone_name.begins_with("f_")) {
			selection_handle->group = Selection::SelectionGroup::FK_FINGER_GROUP;
		} else if (bone_name.ends_with(".L")) {
			selection_handle->group = Selection::SelectionGroup::FK_LEFT_GROUP;
		} else if (bone_name.ends_with(".R")) {
			selection_handle->group = Selection::SelectionGroup::FK_RIGHT_GROUP;
		} else {
			selection_handle->group = Selection::SelectionGroup::FK_CENTER_GROUP;
		}
		selection_handles.push_back(selection_handle);
	}
}

void EPASAnimationEditor::_update_editing_handle_trf() {
	if (editing_selection_handles.size() > 0) {
		Transform3D handle_trf;
		const Ref<Selection> handle = selection_handles[editing_selection_handles[0]];
		switch (handle->type) {
			case (Selection::SelectionType::IK_HANDLE):
			case (Selection::SelectionType::FK_BONE): {
				Ref<EPASPose> pose = get_current_pose();
				handle_trf = editing_skeleton->get_bone_global_pose(handle->bone_idx);
				handle_trf = editing_skeleton->get_global_transform() * handle_trf;
			} break;
			case (Selection::SelectionType::WARP_POINT): {
				handle_trf = handle->warp_point->get_transform();
			} break;
		}

		HBTransformConversions::trf_to_mat(handle_trf, current_handle_trf_matrix.ptrw());
		prev_handle_trf_matrix = current_handle_trf_matrix.duplicate();
		queue_redraw();
	}
}

void EPASAnimationEditor::unhandled_input(const Ref<InputEvent> &p_event) {
	const Ref<InputEventKey> &kev = p_event;
	if (kev.is_valid() && kev->is_pressed() && !kev->is_echo()) {
		if (kev->get_modifiers_mask().has_flag(KeyModifierMask::CTRL)) {
			if (kev->get_keycode() == Key::Z) {
				undo_redo->undo();
			} else if (kev->get_keycode() == Key::Y) {
				undo_redo->redo();
			}
		} else if (kev->get_keycode() == Key::W) {
			guizmo_operation = ImGuizmo::OPERATION::TRANSLATE;
		} else if (kev->get_keycode() == Key::R) {
			guizmo_operation = ImGuizmo::OPERATION::ROTATE;
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
				if (!Input::get_singleton()->is_key_pressed(Key::SHIFT)) {
					editing_selection_handles.clear();
				}
				if (Input::get_singleton()->is_key_pressed(Key::CTRL) && currently_hovered_selection_handle != -1) {
					if (editing_selection_handles.has(currently_hovered_selection_handle)) {
						return;
					}
					editing_selection_handles.push_back(currently_hovered_selection_handle);
					_update_editing_handle_trf();
					accept_event();
				}
			}
		}
	}
}

void EPASAnimationEditor::set_animation(const Ref<EPASAnimation> &p_animation) {
	current_animation = p_animation;
	epas_animation_node->set_animation(p_animation);
	_rebuild_keyframe_cache();
	for (int i = 0; i < p_animation->get_warp_point_count(); i++) {
		_add_warp_point(p_animation->get_warp_point(i));
	}
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
	Array group_vis;
	for (int i = 0; i < ui_info.group_visibility.size(); i++) {
		group_vis.push_back(ui_info.group_visibility[i]);
	}
	current_animation->set_meta("__editor_group_visibility", group_vis);
	current_animation->clear_keyframes();
	Ref<EPASAnimation> editor_animation = current_animation->get_editor_animation();

	const int constexpr static ANIMATION_SAMPLE_FRAMERATE = 30;

	const float animation_length = editor_animation->get_length();
	int animation_sample_count = animation_length * ANIMATION_SAMPLE_FRAMERATE;
	for (int i = 0; i < animation_sample_count; i++) {
		float time = (i / (float)(animation_sample_count - 1)) * animation_length;
		// Bake animation
		Ref<EPASKeyframe> kf;
		kf.instantiate();
		kf->set_time(time);

		epas_animation_node->seek(time);
		epas_controller->advance(0.0f);

		Ref<EPASPose> pose = epas_controller->get_output_pose()->duplicate();

		_apply_constraints(pose);

		kf->set_pose(pose);
		current_animation->add_keyframe(kf);
	}

	int err_code = ResourceSaver::save(current_animation, p_path, ResourceSaver::FLAG_CHANGE_PATH);
	ERR_FAIL_COND_MSG(err_code != OK, vformat("Error saving animation, %s", error_names[err_code]));
	current_animation->set_path(p_path);
}

EPASAnimationEditor::EPASAnimationEditor() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		{
			// Generate the multi mesh used for warp point spheres
			warp_point_spheres = memnew(MultiMeshInstance3D);
			const float SPHERE_RADIUS = 0.1f;
			add_child(warp_point_spheres);

			// Make a sphere with a line pointing forward
			Ref<SphereMesh> sphere;
			sphere.instantiate();
			sphere->set_height(SPHERE_RADIUS * 2.0f);
			sphere->set_radius(SPHERE_RADIUS);
			Array sphere_arrays = sphere->get_mesh_arrays();
			Ref<ArrayMesh> actual_mesh;
			actual_mesh.instantiate();
			actual_mesh->add_surface_from_arrays(sphere->surface_get_primitive_type(0), sphere_arrays);

			Array line_arrays;
			line_arrays.resize(ArrayMesh::ARRAY_MAX);
			PackedVector3Array vertices;
			vertices.push_back(Vector3());
			vertices.push_back(Vector3(0.0, 0.0, -SPHERE_RADIUS));
			line_arrays.set(ArrayMesh::ARRAY_VERTEX, vertices);
			actual_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, line_arrays);

			Ref<StandardMaterial3D> mat;
			mat.instantiate();
			Color col = Color::named("Blue");
			col.a = 0.25f;
			mat->set_albedo(col);
			mat->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
			mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
			actual_mesh->surface_set_material(0, mat);

			Ref<StandardMaterial3D> mat2;
			mat2.instantiate();
			mat2->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
			actual_mesh->surface_set_material(1, mat2);
			Ref<MultiMesh> mm;
			mm.instantiate();
			mm->set_transform_format(MultiMesh::TRANSFORM_3D);
			mm->set_mesh(actual_mesh);
			warp_point_spheres->set_multimesh(mm);
		}

		ui_info.group_visibility.resize(Selection::SelectionGroup::GROUP_MAX);
		ui_info.group_visibility.fill(true);

		solid_ground = memnew(MeshInstance3D);
		add_child(solid_ground);
		solid_ground->set_position(Vector3(0, -0.001, 0));

		Ref<PlaneMesh> plane;
		plane.instantiate();
		plane->set_size(Vector2(10, 10));

		solid_ground->set_mesh(plane);

		// HACK-ish but works, ensures we draw everything behind the imgui UI
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
			placeholder_load_dialog = memnew(FileDialog);
			PackedStringArray filters;
			filters.push_back("*.gltf ; GLTF model");
			filters.push_back("*.blend ; Blender file");
			filters.push_back("*.tscn ; Godot scene file");
			placeholder_load_dialog->set_filters(filters);
			add_child(placeholder_load_dialog);
			placeholder_load_dialog->set_file_mode(FileDialog::FileMode::FILE_MODE_OPEN_FILE);
			placeholder_load_dialog->connect("file_selected", callable_mp(this, &EPASAnimationEditor::_load_placeholder_scene));
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
		HBTransformConversions::trf_to_mat(Transform3D(), identity_matrix.ptrw());

		camera = memnew(EPASEditorCamera);
		editor_3d_root->add_child(camera);
		camera->set_position(Vector3(0.0, 1.0, 2.0));

		DirectionalLight3D *dl = memnew(DirectionalLight3D);
		dl->set_shadow(true);
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

		set_process_unhandled_input(true);

		// Setup animation display

		epas_controller = memnew(EPASController);
		add_child(epas_controller, true);

		epas_animation_node.instantiate();
		epas_animation_node->set_playback_mode(EPASAnimationNode::PlaybackMode::MANUAL);
		epas_animation_node->set_looping_enabled(false);
		epas_blend_node.instantiate();

		epas_controller->connect_node_to_root(epas_blend_node, "Blend Node");
		epas_controller->connect_node(epas_animation_node, epas_blend_node, "Animation", 0);

		epas_pose_node.instantiate();
		epas_controller->connect_node(epas_pose_node, epas_blend_node, "IK Blend", 1);

		undo_redo = memnew(UndoRedo);
		Ref<EPASEditorAnimation> editor_animation;
		editor_animation.instantiate();
		editor_animation->set_editor_animation(memnew(EPASAnimation));
		set_animation(editor_animation);
		curves_editor.instantiate();
		events_editor.instantiate();
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

void EPASAnimationCurvesEditor::draw(Ref<EPASAnimation> p_animation, float p_playback_position) {
	if (!p_animation.is_valid()) {
		return;
	}

	if (!open) {
		return;
	}

	HashMap<StringName, Ref<Curve>> curves = p_animation->get_animation_curves();

	if (ImGui::Begin("Curves Editor", &open)) {
		if (ImGui::BeginCombo("Curve", String(current_curve).utf8().get_data())) {
			for (KeyValue<StringName, Ref<Curve>> kv : curves) {
				if (ImGui::Selectable(String(kv.key).utf8().get_data())) {
					current_curve = kv.key;
				}
			}
			ImGui::EndCombo();
		}
		ImGui::SameLine();
		if (ImGui::Button("Add")) {
			ImGui::OpenPopup("Create curve");
		}

		if (ImGui::BeginPopupModal("Create curve", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
			ImGui::TextUnformatted("Enter your new item's name.");
			static char buf[200];
			ImGui::InputText("##curvename", buf, 200);
			if (ImGui::Button("Ok", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
				Ref<Curve> crv;
				crv.instantiate();
				p_animation->insert_animation_curve(String(buf), crv);
				ImGui::CloseCurrentPopup();
			}
			ImGui::SetItemDefaultFocus();
			ImGui::EndPopup();
		}

		if (!curves.has(current_curve)) {
			ImGui::End();
			return;
		}

		ImDrawList *dl = ImGui::GetWindowDrawList();
		ImVec2 cursor_pos = ImGui::GetCursorScreenPos();
		Ref<Curve> curve = curves[current_curve];
		ImVec2 avail = ImGui::GetContentRegionAvail();
		avail.y = 200.0f;
		dl->AddRectFilled(cursor_pos, Vector2(cursor_pos) + Vector2(avail), IM_COL32_BLACK);
		for (int i = 0; i < curve->get_point_count() - 1; i++) {
			Vector2 p1 = curve->get_point_position(i);
			Vector2 p4 = curve->get_point_position(i + 1);
			float dst = p4.x - p1.x;
			dst /= 3.0f;
			Vector2 p2 = Vector2(p1.x + dst, p1.y + curve->get_point_right_tangent(i) * p1.y);
			Vector2 p3 = Vector2(p4.x - dst, p4.y + curve->get_point_left_tangent(i + 1) * p4.y);

			p1.y = 1.0f - p1.y;
			p2.y = 1.0f - p2.y;
			p3.y = 1.0f - p3.y;
			p4.y = 1.0f - p4.y;

			p1 *= avail;
			p2 *= avail;
			p3 *= avail;
			p4 *= avail;

			p1 += ImGui::GetCursorScreenPos();
			p2 += ImGui::GetCursorScreenPos();
			p3 += ImGui::GetCursorScreenPos();
			p4 += ImGui::GetCursorScreenPos();

			dl->AddBezierCubic(p1, p2, p3, p4, IM_COL32_WHITE, 1.0f);
		}
		dl->AddLine(
				Vector2(cursor_pos) + Vector2(avail.x * p_playback_position, 0.0f),
				Vector2(cursor_pos) + Vector2(avail.x * p_playback_position, avail.y),
				IM_COL32(255, 165, 0, 255));
		ImGui::Dummy(avail);
		if (ImGui::BeginTable("Tables", 2, ImGuiTableFlags_Borders)) {
			for (int i = 0; i < curve->get_point_count(); i++) {
				ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
				ImGui::TableNextColumn();
				ImGui::Text("Point %d", i);

				ImGui::PushID(i);
				if (i != 0) {
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::TextUnformatted("Left tangent");
					ImGui::TableNextColumn();
					float left_tangent = curve->get_point_left_tangent(i);

					if (ImGui::InputFloat("##LeftTangent", &left_tangent)) {
						curve->set_point_left_tangent(i, left_tangent);
					}
				}

				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::TextUnformatted("Position");
				ImGui::TableNextColumn();
				Vector2 position = curve->get_point_position(i);
				float pos[2] = { position.x, position.y };
				if (ImGui::InputFloat2("##position", pos)) {
					curve->set_point_offset(i, pos[0]);
					curve->set_point_value(i, pos[1]);
				}

				if (i != curve->get_point_count() - 1) {
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::TextUnformatted("Right tangent");
					ImGui::TableNextColumn();
					float right_tangent = curve->get_point_right_tangent(i);

					if (ImGui::InputFloat("##RightTangent", &right_tangent)) {
						curve->set_point_right_tangent(i, right_tangent);
					}
				}
				ImGui::PopID();
			}
			ImGui::EndTable();
		}
		if (ImGui::Button("Add2")) {
			curve->add_point(Vector2(p_playback_position, 0.0f));
		}
	}
	ImGui::End();
}

void EPASEditorAnimationNode::interpolate(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_time) {
	Ref<EPASEditorAnimation> anim = get_animation();
	ERR_FAIL_COND(!anim.is_valid());

	// UGLY HACK: So events work... maybe we have to rethink the whole editor animation system or something.
	if (anim->get_event_count() != anim->get_editor_animation()->get_event_count()) {
		anim->get_editor_animation()->clear_events();
		for (int i = 0; i < anim->get_event_count(); i++) {
			anim->get_editor_animation()->add_event(anim->get_event(i));
		}
	}

	if (!anim->get_editor_animation()->is_connected(SNAME("event_fired"), callable_mp((EPASAnimationNode *)this, &EPASEditorAnimationNode::_on_animation_event_fired))) {
		anim->get_editor_animation()->connect(SNAME("event_fired"), callable_mp((EPASAnimationNode *)this, &EPASEditorAnimationNode::_on_animation_event_fired));
	}
	anim->get_editor_animation()->interpolate(p_time, p_base_pose, p_target_pose, get_interpolation_method(), &playback_info);
	playback_info.last_time = p_time;
}

String EPASAnimationEventsEditor::get_event_name(const Ref<EPASAnimationEvent> &ev) const {
	if (!ev.is_valid()) {
		return "None";
	}
	Ref<EPASSoundAnimationEvent> sound_ev = ev;
	if (sound_ev.is_valid()) {
		Ref<AudioStream> stream = sound_ev->get_stream();
		if (stream.is_valid()) {
			return vformat("Sound event %s", stream->get_path().get_file());
		} else {
			return "Sound event";
		}
	}

	return ev->get_class();
}

void EPASAnimationEventsEditor::_on_sound_file_selected(String p_file) {
	if (Ref<EPASSoundAnimationEvent> sev = current_event; sev.is_valid()) {
		sev->set_stream(ResourceLoader::load(p_file));
	}
}

void EPASAnimationEventsEditor::draw(Ref<EPASAnimation> p_animation, float p_playback_position) {
	if (!p_animation.is_valid()) {
		return;
	}

	if (!open) {
		return;
	}

	if (ImGui::Begin("Events Editor", &open)) {
		if (ImGui::BeginCombo("Events", get_event_name(current_event).utf8().get_data())) {
			for (int i = 0; i < p_animation->get_event_count(); i++) {
				if (ImGui::Selectable(get_event_name(p_animation->get_event(i)).utf8().get_data())) {
					current_event = p_animation->get_event(i);
				}
			}
			ImGui::EndCombo();
		}
		ImGui::SameLine();
		if (ImGui::Button("Add sound")) {
			Ref<EPASSoundAnimationEvent> sound_ev;
			sound_ev.instantiate();
			p_animation->add_event(sound_ev);
		}

		if (current_event.is_valid()) {
			float time = current_event->get_time();
			if (ImGui::InputFloat("Time", &time)) {
				current_event->set_time(time);
			}

			ImGui::SameLine();

			if (ImGui::Button("Set current")) {
				current_event->set_time(p_playback_position);
			}

			Ref<EPASSoundAnimationEvent> sound_ev = current_event;
			if (sound_ev.is_valid()) {
				Ref<AudioStream> as = sound_ev->get_stream();
				if (!as.is_valid()) {
					ImGui::TextUnformatted("No audio");
				} else {
					ImGui::TextUnformatted(as->get_path().utf8().get_data());
				}
				ImGui::SameLine();
				if (ImGui::Button("Select...")) {
					sound_file_dialog->popup_centered_ratio();
				}
			}
		}
	}
	ImGui::End();
}

EPASAnimationEventsEditor::EPASAnimationEventsEditor() {
	sound_file_dialog = memnew(FileDialog);
	Vector<String> filters;
	filters.push_back("*.wav; Audio files");
	sound_file_dialog->set_filters(filters);
	sound_file_dialog->connect("file_selected", callable_mp(this, &EPASAnimationEventsEditor::_on_sound_file_selected));
	sound_file_dialog->set_file_mode(FileDialog::FILE_MODE_OPEN_FILE);
	SceneTree::get_singleton()->get_root()->add_child(sound_file_dialog);
}

EPASAnimationEventsEditor::~EPASAnimationEventsEditor() {
	sound_file_dialog->queue_free();
	sound_file_dialog = nullptr;
}

#endif