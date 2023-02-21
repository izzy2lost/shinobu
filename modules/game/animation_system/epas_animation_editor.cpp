#ifdef DEBUG_ENABLED
#include "epas_animation_editor.h"
#include "../utils.h"
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
#include "modules/imgui/godot_imgui.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/light_3d.h"
#include "scene/3d/world_environment.h"
#include "scene/resources/packed_scene.h"
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
			epas_animation_node->seek(current_frame / (float)FPS);
		} break;
		case NOTIFICATION_DRAW: {
			if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
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
		HashMap<int, Vector2> bones_screen_position;
		bones_screen_position.reserve(editing_skeleton->get_bone_count());
		Vector2 mouse_pos = get_global_mouse_position();
		int closest_bone = -1;
		Vector2 closest_bone_scr_pos;
		for (int i = 0; i < editing_skeleton->get_bone_count(); i++) {
			Vector3 bone_pos = editing_skeleton->to_global(editing_skeleton->get_bone_global_pose(i).origin);
			if (!cam->is_position_behind(bone_pos)) {
				Vector2 bone_scr_pos = cam->unproject_position(bone_pos);
				if (mouse_pos.distance_to(bone_scr_pos) < 25.0) {
					if (closest_bone == -1 || mouse_pos.distance_to(bone_scr_pos) < mouse_pos.distance_to(closest_bone_scr_pos)) {
						closest_bone = i;
						closest_bone_scr_pos = bone_scr_pos;
					}
				}

				bones_screen_position.insert(i, bone_scr_pos);
			}
		}
		Color color_unlit = Color::named("Dark Magenta");
		Color color_lit = Color::named("Light Yellow");

		for (const KeyValue<int, Vector2> &pair : bones_screen_position) {
			if (pair.key == closest_bone) {
				draw_circle(pair.value, 4.0, color_lit);
				draw_string(label_font, pair.value + Vector2(10.0, 0.0), editing_skeleton->get_bone_name(pair.key));
			} else {
				draw_circle(pair.value, 4.0, color_unlit);
			}
		}
		currently_hovered_bone = closest_bone;
	}
}

void EPASAnimationEditor::_world_to_bone_trf(int p_bone_idx, const float *p_world_trf, Transform3D &p_out) {
	HBUtils::mat_to_trf(p_world_trf, p_out);
	p_out = editing_skeleton->get_global_transform().affine_inverse() * p_out;
	int parent = editing_skeleton->get_bone_parent(editing_bone);
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

// Returns the current pose that the user is hovering, if any
Ref<EPASPose> EPASAnimationEditor::get_current_pose() const {
	for (int i = 0; i < keyframe_cache.size(); i++) {
		if ((uint32_t)keyframe_cache[i]->frame_time == current_frame) {
			return keyframe_cache[i]->keyframe->get_pose();
		}
	}
	return Ref<EPASPose>();
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
					current_pose->flip_along_z();
				}
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
		ImVec2 window_pos;
		ImVec2 work_pos = ImGui::GetMainViewport()->WorkPos;
		window_pos.x = work_pos.x + 10.0f;
		window_pos.y = work_pos.y + 10.0f;
		ImGui::SetNextWindowBgAlpha(0.35);
		ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, ImVec2(0.0f, 0.0f));
		ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize;
		if (ImGui::Begin("ToolDock", nullptr, window_flags)) {
			ImGui::RadioButton("Global", (int *)&guizmo_mode, (int)ImGuizmo::WORLD);
			ImGui::SameLine();
			ImGui::RadioButton("Local", (int *)&guizmo_mode, (int)ImGuizmo::LOCAL);
			ImGui::SameLine();
			ImGui::Separator();
			ImGui::SameLine();
			ImGui::RadioButton("Translate", (int *)&guizmo_operation, (int)ImGuizmo::TRANSLATE);
			ImGui::SameLine();
			ImGui::RadioButton("Rotate", (int *)&guizmo_operation, (int)ImGuizmo::ROTATE);
			ImGui::SameLine();
			ImGui::RadioButton("Scale", (int *)&guizmo_operation, (int)ImGuizmo::SCALE);
		}
		ImGui::End();
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
		ImGui::DockBuilderFinish(dock_id);
	}
	ImGui::DockSpace(dock_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
	ImGui::PopStyleVar();
	ImGui::End();

	if (ImGui::Begin("Timeline")) {
		ImGuiNeoSequencerFlags sequencer_flags = ImGuiNeoSequencerFlags_EnableSelection;
		sequencer_flags |= ImGuiNeoSequencerFlags_Selection_EnableDragging;
		sequencer_flags |= ImGuiNeoSequencerFlags_Selection_EnableDeletion;
		uint32_t old_current_frame = current_frame;
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
					undo_redo->commit_action();
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
		ImGui::Combo("Interpolation", reinterpret_cast<int *>(&interp_method), items, IM_ARRAYSIZE(items));
		if (interp_method != original_interp_method) {
			epas_animation_node->set_interpolation_method(interp_method);
		}
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
		if (editing_bone != -1) {
			manipulated = ImGuizmo::Manipulate(view_matrix.ptr(), projection_matrix.ptr(), guizmo_operation, guizmo_mode, editing_bone_trf_matrix.ptrw());
			if (manipulated) {
				Transform3D bone_trf;
				_world_to_bone_trf(editing_bone, editing_bone_trf_matrix.ptr(), bone_trf);
				String bone_name = editing_skeleton->get_bone_name(editing_bone);
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
			}

			if (!ImGuizmo::IsUsing() && was_using_gizmo) {
				String bone_name = editing_skeleton->get_bone_name(editing_bone);
				Transform3D new_bone_trf;
				Transform3D old_bone_trf;
				_world_to_bone_trf(editing_bone, editing_bone_trf_matrix.ptr(), new_bone_trf);
				_world_to_bone_trf(editing_bone, selected_bone_trf_matrix.ptr(), old_bone_trf);
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
				undo_redo->add_do_method(callable_mp(this, &EPASAnimationEditor::_update_editing_skeleton_trf));
				undo_redo->add_undo_method(callable_mp(this, &EPASAnimationEditor::_update_editing_skeleton_trf));
				undo_redo->commit_action();
			}
			was_using_gizmo = ImGuizmo::IsUsing();
		}
	}
}

void EPASAnimationEditor::open_file(const String &p_path) {
	Ref<EPASAnimation> animation = ResourceLoader::load(p_path);
	if (animation.is_valid()) {
		undo_redo->clear_history();
		if (animation->has_meta("__editor_model_path")) {
			String model_path = animation->get_meta("__editor_model_path");
			load_model(model_path);
		}
		set_animation(animation);
		undo_redo->clear_history();
	} else {
		// TODO: Error handling here
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
	// TODO: Show error to user
	ERR_FAIL_COND(scene.is_null());

	Node *node = scene->instantiate();
	Node3D *node_3d = Object::cast_to<Node3D>(node);
	TypedArray<Node> skel_candidate = node_3d ? node_3d->find_children("*", "Skeleton3D") : TypedArray<Node>();
	Skeleton3D *skel = nullptr;
	if (skel_candidate.size() > 0) {
		skel = Object::cast_to<Skeleton3D>(skel_candidate[0]);
	}
	if (!skel) {
		if (node) {
			node->queue_free();
		}
		return;
	}

	add_child(node_3d);
	epas_controller->set_skeleton_path(skel->get_path());
	editing_skeleton = skel;
	current_model = node_3d;
}

void EPASAnimationEditor::_update_editing_skeleton_trf() {
	if (editing_bone != -1) {
		Transform3D bone_trf = editing_skeleton->get_bone_global_pose(editing_bone);
		bone_trf = editing_skeleton->get_global_transform() * bone_trf;
		HBUtils::trf_to_mat(bone_trf, editing_bone_trf_matrix.ptrw());
		selected_bone_trf_matrix = editing_bone_trf_matrix.duplicate();
		queue_redraw();
	}
}

void EPASAnimationEditor::unhandled_input(const Ref<InputEvent> &p_event) {
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
				editing_bone = -1;
				if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
					editing_bone = currently_hovered_bone;
					if (editing_bone != -1) {
						_update_editing_skeleton_trf();
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
	int err_code = ResourceSaver::save(current_animation, p_path, ResourceSaver::FLAG_CHANGE_PATH);
	ERR_FAIL_COND_MSG(err_code != OK, vformat("Error saving animation, %s", error_names[err_code]));
}

EPASAnimationEditor::EPASAnimationEditor() {
	if (!Engine::get_singleton()->is_editor_hint()) {
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
		editing_bone_trf_matrix.resize_zeroed(16);
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

		set_process_unhandled_input(true);

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