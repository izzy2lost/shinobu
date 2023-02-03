#ifdef DEBUG_ENABLED
#include "epas_pose_editor.h"

#include "modules/imgui/godot_imgui.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/light_3d.h"
#include "scene/3d/world_environment.h"
#include "scene/main/canvas_layer.h"
#include "scene/resources/packed_scene.h"
#include "scene/resources/sky_material.h"

void trf_to_mat(const Transform3D &p_mat, float *p_out) {
	p_out[0] = p_mat.basis.rows[0][0];
	p_out[1] = p_mat.basis.rows[1][0];
	p_out[2] = p_mat.basis.rows[2][0];
	p_out[3] = 0;

	p_out[4] = p_mat.basis.rows[0][1];
	p_out[5] = p_mat.basis.rows[1][1];
	p_out[6] = p_mat.basis.rows[2][1];
	p_out[7] = 0;

	p_out[8] = p_mat.basis.rows[0][2];
	p_out[9] = p_mat.basis.rows[1][2];
	p_out[10] = p_mat.basis.rows[2][2];
	p_out[11] = 0;

	p_out[12] = p_mat.origin.x;
	p_out[13] = p_mat.origin.y;
	p_out[14] = p_mat.origin.z;
	p_out[15] = 1;
}

void mat_to_trf(const float *p_in, Transform3D &p_out) {
	p_out.basis.rows[0][0] = p_in[0];
	p_out.basis.rows[1][0] = p_in[1];
	p_out.basis.rows[2][0] = p_in[2];

	p_out.basis.rows[0][1] = p_in[4];
	p_out.basis.rows[1][1] = p_in[5];
	p_out.basis.rows[2][1] = p_in[6];

	p_out.basis.rows[0][2] = p_in[8];
	p_out.basis.rows[1][2] = p_in[9];
	p_out.basis.rows[2][2] = p_in[10];

	p_out.origin.x = p_in[12];
	p_out.origin.y = p_in[13];
	p_out.origin.z = p_in[14];
}

void proj_to_mat(const Projection &p_mat, float *p_out) {
	for (int i = 0; i < 16; i++) {
		real_t *p = (real_t *)p_mat.columns;
		p_out[i] = p[i];
	}
}

Transform3D EPASPoseEditor::get_edited_object_transform() {
	return Transform3D();
}

void EPASPoseEditor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			label_font = get_theme_font(SNAME("font"));
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			_draw_ui();
		} break;
		case NOTIFICATION_DRAW: {
			if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
				_draw_bone_positions();
			}
		} break;
	}
}

void EPASPoseEditor::_draw_bone_positions() {
	Camera3D *cam = get_viewport()->get_camera_3d();
	if (cam && editing_skeleton) {
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

void EPASPoseEditor::_draw_ui() {
	ImGuiIO io = ImGui::GetIO();
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Open", "CTRL+O")) {
				file_open_dialog->popup_centered_ratio(0.75);
			}
			if (ImGui::MenuItem("Load model", "CTRL+L")) {
				model_load_dialog->popup_centered_ratio(0.75);
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
		ImVec2 window_pos;
		ImVec2 work_pos = ImGui::GetMainViewport()->WorkPos;
		ImVec2 work_size = ImGui::GetMainViewport()->WorkSize;
		window_pos.x = work_pos.x + work_size.x - 10.0f;
		window_pos.y = work_pos.y + 10.0f;
		ImGui::SetNextWindowBgAlpha(0.35);
		ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, ImVec2(1.0f, 0.0f));
		ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize;
		if (ImGui::Begin("ToolDock", nullptr, window_flags)) {
			ImGui::RadioButton("Global", (int *)&guizmo_mode, (int)ImGuizmo::WORLD);
			ImGui::SameLine();
			ImGui::RadioButton("Local", (int *)&guizmo_mode, (int)ImGuizmo::LOCAL);
		}
		ImGui::End();
	}
	Ref<World3D> world = editor_3d_root->get_world_3d();

	if (world.is_valid()) {
		Camera3D *cam = get_viewport()->get_camera_3d();
		if (!cam) {
			return;
		}

		Transform3D view_trf = cam->get_global_transform().inverse();
		trf_to_mat(view_trf, view_matrix.ptrw());

		Projection proj;
		float aspect = io.DisplaySize.x / io.DisplaySize.y;
		proj.set_perspective(cam->get_fov(), aspect, cam->get_near(), cam->get_far(), cam->get_keep_aspect_mode() == Camera3D::KEEP_WIDTH);
		proj_to_mat(proj, projection_matrix.ptrw());

		ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
		if (editing_bone != -1) {
			bool manipulated = ImGuizmo::Manipulate(view_matrix.ptr(), projection_matrix.ptr(), guizmo_operation, guizmo_mode, editing_bone_trf_matrix.ptrw());
			if (manipulated) {
				Transform3D bone_trf;
				mat_to_trf(editing_bone_trf_matrix.ptr(), bone_trf);
				bone_trf = editing_skeleton->get_global_transform().affine_inverse() * bone_trf;
				int parent = editing_skeleton->get_bone_parent(editing_bone);
				if (parent != -1) {
					bone_trf = editing_skeleton->get_bone_global_pose(parent).affine_inverse() * bone_trf;
				}
				String bone_name = editing_skeleton->get_bone_name(editing_bone);
				if (!current_pose->get_bone_data(bone_name)) {
					current_pose->create_bone(bone_name);
				}
				switch (guizmo_operation) {
					case ImGuizmo::TRANSLATE: {
						current_pose->set_bone_position(bone_name, bone_trf.origin);
						//undo_redo->create_action("Translate bone");
					} break;
					case ImGuizmo::ROTATE: {
						current_pose->set_bone_rotation(bone_name, bone_trf.basis.get_rotation_quaternion());
						//undo_redo->create_action("Rotate bone");
					} break;
					case ImGuizmo::SCALE: {
						current_pose->set_bone_scale(bone_name, bone_trf.basis.get_scale());
						//undo_redo->create_action("Scale bone");
					} break;
					default: {
					};
				}
			}
		}
	}
}

void EPASPoseEditor::open_file(const String &p_path) {
	Ref<EPASPose> pose = ResourceLoader::load(p_path);
	if (pose.is_valid()) {
		set_pose(current_pose);
	}
}

void EPASPoseEditor::load_model(const String &path) {
	if (editing_model) {
		editing_model->queue_free();
		editing_model = nullptr;
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
	editing_model = node_3d;
}

void EPASPoseEditor::unhandled_input(const Ref<InputEvent> &p_event) {
	const Ref<InputEventMouseMotion> &mev = p_event;
	if (mev.is_valid()) {
		if (mev->get_relative().length() > 0) {
			queue_redraw();
		}
	}
	const Ref<InputEventMouseButton> &bev = p_event;
	if (bev.is_valid()) {
		if (bev->is_pressed() && bev->get_button_index() == MouseButton::LEFT) {
			editing_bone = -1;
			if (Input::get_singleton()->is_key_pressed(Key::CTRL)) {
				editing_bone = currently_hovered_bone;
				if (editing_bone != -1) {
					Transform3D bone_trf = editing_skeleton->get_bone_global_pose(editing_bone);
					bone_trf = editing_skeleton->get_global_transform() * bone_trf;
					trf_to_mat(bone_trf, editing_bone_trf_matrix.ptrw());
				}
			}
		}
	}
}

void EPASPoseEditor::set_pose(const Ref<EPASPose> &p_pose) {
	current_pose = p_pose;
	epas_pose_node->set_pose(current_pose);
}

EPASPoseEditor::EPASPoseEditor() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		editor_3d_root = memnew(Node3D);
		add_child(editor_3d_root);
		{
			file_open_dialog = memnew(FileDialog);
			PackedStringArray filters;
			filters.push_back("*.epos ; EPAS Pose");
			file_open_dialog->set_filters(filters);
			add_child(file_open_dialog);
			file_open_dialog->connect("file_selected", callable_mp(this, &EPASPoseEditor::open_file));
		}
		{
			model_load_dialog = memnew(FileDialog);
			PackedStringArray filters;
			filters.push_back("*.gltf ; GLTF model");
			filters.push_back("*.blend ; Blender file");
			model_load_dialog->set_filters(filters);
			add_child(model_load_dialog);
			model_load_dialog->set_file_mode(FileDialog::FileMode::FILE_MODE_OPEN_FILE);
			model_load_dialog->connect("file_selected", callable_mp(this, &EPASPoseEditor::load_model));
		}

		set_process_internal(true);
		// Setup 3d buffer and identity matrix
		view_matrix.resize_zeroed(16);
		projection_matrix.resize_zeroed(16);
		editing_bone_trf_matrix.resize_zeroed(16);
		identity_matrix.resize_zeroed(16);
		trf_to_mat(Transform3D(), identity_matrix.ptrw());

		camera = memnew(EPASEditorCamera);
		editor_3d_root->add_child(camera);
		camera->set_position(Vector3(0.0, 1.0, 0.0));

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

			wenv->set_environment(env);
			add_child(wenv);
		}

		set_process_unhandled_input(true);

		epas_controller = memnew(EPASController);
		add_child(epas_controller, true);
		epas_pose_node = Ref<EPASPoseNode>(memnew(EPASPoseNode));
		epas_controller->connect_node_to_root(epas_pose_node, "Pose");

		undo_redo = memnew(UndoRedo);
		set_pose(memnew(EPASPose));
	}
}

EPASPoseEditor::~EPASPoseEditor() {
	if (undo_redo) {
		memdelete(undo_redo);
	}
}

#endif