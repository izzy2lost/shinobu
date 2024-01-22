#include "epas_controller.h"

#include "core/variant/array.h"
#include "core/variant/variant.h"
#include "modules/game/animation_system/epas_animation_node.h"
#ifdef DEBUG_ENABLED
#include "imgui.h"
#include "implot.h"
#include "modules/imgui/godot_imgui.h"
#include "modules/imgui/godot_imgui_macros.h"
#include "modules/imgui/thirdparty/imgui/imnodes.h"
#endif
#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/immediate_mesh.h"
#include "scene/resources/material.h"
#include "scene/resources/mesh.h"

#ifdef DEBUG_ENABLED
static bool has_finished_init = false;
static bool node_position_changed = false;
#endif

void EPASController::set_playback_process_mode(EPASController::PlaybackProcessMode p_playback_process_mode) {
	playback_process_mode = p_playback_process_mode;
	_update_process_mode();
}

void EPASController::_update_process_mode() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	set_process_internal(playback_process_mode == EPASController::PlaybackProcessMode::IDLE);
	set_physics_process_internal(playback_process_mode == EPASController::PlaybackProcessMode::PHYSICS_PROCESS);
#ifdef DEBUG_ENABLED
	// Debug mode always need process internal for ImGui to update
	set_process_internal(true);
#endif
}

void EPASController::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			advance(get_physics_process_delta_time());
		} break;
		case NOTIFICATION_ENTER_TREE: {
			_update_process_mode();
#ifdef DEBUG_ENABLED
			REGISTER_DEBUG(this);
#endif
		} break;
#ifdef DEBUG_ENABLED
		case NOTIFICATION_EXIT_TREE: {
			UNREGISTER_DEBUG(this);
		} break;
#endif
		case NOTIFICATION_INTERNAL_PROCESS: {
#ifdef DEBUG_ENABLED
			GodotImGui *gim = GodotImGui::get_singleton();
			if (gim && gim->is_debug_enabled(this)) {
				if (gim->begin_debug_window(this)) {
					ImGui::Checkbox("Show skeleton", &debug_enable_skeleton_vis);
					if (output_pose.is_valid() && output_pose->has_bone("spine")) {
						for (int i = 1; i < 90; i++) {
							hip_plot_lines_x.set(i - 1, hip_plot_lines_x[i]);
							hip_plot_lines_y.set(i - 1, hip_plot_lines_y[i]);
						}
						plot_t += get_process_delta_time();
						hip_plot_lines_x.set(90 - 1, plot_t);
						hip_plot_lines_y.set(90 - 1, output_pose->get_bone_position("spine", base_pose_cache).y);
						String title = "Hip Y";
						if (ImPlot::BeginPlot("Hip Y", ImVec2(600.0f, 200.0f), ImPlotFlags_CanvasOnly & ~(ImPlotFlags_NoTitle | ImPlotFlags_NoLegend))) {
							ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels);
							ImPlot::SetupAxisLimits(ImAxis_X1, hip_plot_lines_x[0], hip_plot_lines_x[hip_plot_lines_x.size() - 1], ImGuiCond_Always);
							ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0f, 1.0f);
							ImPlot::PlotLine("Hip Y", hip_plot_lines_x.ptr(), hip_plot_lines_y.ptr(), hip_plot_lines_x.size());
							ImPlot::EndPlot();
						}
					}

					ImGui::Text("Node count %d", nodes.size());
					ImGui::SameLine();
					if (ImGui::Button("Arrange")) {
						_arrange_nodes();
					}
					ImNodes::BeginNodeEditor();
					debug_draw_accumulator = 0;
					debug_draw_link_accumulator = 0;
					debug_draw_attrib_accumulator = 0;
					_debug_draw_node(root, nullptr);
					if (!has_finished_init) {
						has_finished_init = true;
					}
					if (node_position_changed && ImGui::IsMouseReleased(0)) {
						node_position_changed = false;
						gim->save_config();
					}
					ImNodes::EndNodeEditor();
				}
				ImGui::End();
			}
#endif
			if (playback_process_mode != PlaybackProcessMode::IDLE) {
				return;
			}
			advance(get_process_delta_time());
		} break;
	}
}

#ifdef DEBUG_ENABLED
void EPASController::_debug_draw_node(Ref<EPASNode> p_node, int *p_output_attrib_id) {
	int64_t object_id = p_node->get_instance_id();
	if (!has_finished_init) {
		StringName epas_name = p_node->get_meta("epas_name");
		String pos_property_name = String(epas_name) + "/position";
		p_node->set_meta("epas_pos_property_name", pos_property_name);
		Vector2 pos = GodotImGui::get_singleton()->get_config_value(this, pos_property_name, Vector2());
		p_node->set_meta("epas_pos", pos);
		ImNodes::SetNodeEditorSpacePos(object_id, ImVec2(pos.x, pos.y));
	} else {
		/*ImVec2 curr_pos = ImNodes::GetNodeEditorSpacePos((int32_t)object_id);
		Vector2 new_pos = Vector2(curr_pos.x, curr_pos.y);
		Vector2 prev_pos = p_node->get_meta("epas_pos");
		if (prev_pos != new_pos) {
			p_node->set_meta("epas_pos", new_pos);
			GodotImGui::get_singleton()->set_config_value(this, p_node->get_meta("epas_pos_property_name"), new_pos);
			node_position_changed = true;
		}*/
	}
	ImNodes::BeginNode((int32_t)object_id);
	// Root node doesn't have an output
	ImNodes::BeginNodeTitleBar();
	String t = p_node->get_meta("epas_name", String());
	ImGui::TextUnformatted(t.utf8().ptr());
	ImNodes::EndNodeTitleBar();

	Ref<EPASRootNode> r = p_node;
	if (!r.is_valid()) {
		int output_link_id = debug_draw_attrib_accumulator;
		*p_output_attrib_id = output_link_id;

		ImNodes::BeginOutputAttribute(debug_draw_attrib_accumulator++);
		ImNodes::EndOutputAttribute();
	}

	int input_attrib_start = debug_draw_attrib_accumulator;

	for (int i = 0; i < p_node->get_input_count(); i++) {
		ImNodes::BeginInputAttribute(debug_draw_attrib_accumulator++);
		ImGui::Text("Input %d", i);
		ImNodes::EndInputAttribute();
	}

	p_node->_debug_node_draw();

	ImNodes::EndNode();

	for (int i = 0; i < p_node->get_input_count(); i++) {
		int attrib_out;
		Ref<EPASNode> n = p_node->get_input(i);
		if (n.is_null()) {
			continue;
		}
		_debug_draw_node(n, &attrib_out);
		ImNodes::Link(debug_draw_link_accumulator++, attrib_out, input_attrib_start + i);
	}
}
int EPASController::_get_skeleton_line_count(Skeleton3D *p_skel) {
	int count = 0;
	for (int i = 0; i < p_skel->get_bone_count(); i++) {
		count += p_skel->get_bone_children(i).size() * 2;
	}
	return count;
}
void EPASController::_debug_update_skeleton_vis() {
	Skeleton3D *skel = get_skeleton();
	if (!skel) {
		return;
	}
	if (!debug_skeleton_vis) {
		debug_skeleton_vis = memnew(MeshInstance3D);
		debug_skeleton_vis->set_mesh(memnew(ArrayMesh));
		add_child(debug_skeleton_vis);
		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
		mat->set_albedo(Color(1.0, 1.0, 0.0, 1.0));
		mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
		debug_skeleton_vis->set_material_override(mat);
	}
	debug_skeleton_vis->show();
	Ref<ArrayMesh> mesh = debug_skeleton_vis->get_mesh();
	int mesh_bone_count = mesh->get_meta("mesh_bone_count", -1);
	if (mesh_bone_count != skel->get_bone_count()) {
		// Recreate basic arrays
		Array a;
		a.resize(Mesh::ARRAY_MAX);
		int vertex_count = _get_skeleton_line_count(skel);
		skel_dbg_vertex_array.resize_zeroed(vertex_count);
		a[Mesh::ARRAY_VERTEX] = skel_dbg_vertex_array;
		mesh->clear_surfaces();
		mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, a);
	}

	int updated_lines = 0;

	for (int i = 0; i < skel->get_bone_count(); i++) {
		Vector<int> children = skel->get_bone_children(i);
		for (int ii = 0; ii < children.size(); ii++) {
			int child_bone_i = children[ii];
			skel_dbg_vertex_array.set(updated_lines * 2, skel->get_bone_global_pose(i).origin);
			skel_dbg_vertex_array.set(updated_lines * 2 + 1, skel->get_bone_global_pose(child_bone_i).origin);
			updated_lines++;
		}
	}

	mesh->surface_update_vertex_region(0, 0, skel_dbg_vertex_array.to_byte_array());
	debug_skeleton_vis->set_global_transform(skel->get_global_transform());
}
void EPASController::_arrange_nodes() {
	Ref<EPASNode> root_node = get_epas_node("Output");
	Vector<Vector<Ref<EPASNode>>> grouped_nodes;
	grouped_nodes.resize_zeroed(32);

	int depth_count = get_nodes_by_depth(root_node, grouped_nodes);

	float depth_width_accumulator = 0.0f;
	const float VERTICAL_MARGIN = 25.0f;
	const float HORIZONTAL_MARGIN = 50.0f;
	for(int i = 0; i < depth_count; i++) {
		float this_depth_width = 0.0f;
		float this_depth_height = 0.0f;
		float height_accumulator = 0.0f;
		for (int j = 0; j < grouped_nodes[i].size(); j++) {
			int64_t obj_id = grouped_nodes[i][j]->get_instance_id();
			this_depth_width = MAX(ImNodes::GetNodeDimensions(obj_id).x, this_depth_width);
			this_depth_height += ImNodes::GetNodeDimensions(obj_id).y;

			if (j != grouped_nodes[i].size()-1) {
				this_depth_height += VERTICAL_MARGIN;
			}
		}
		for (int j = 0; j < grouped_nodes[i].size(); j++) {
			Vector2 node_pos;
			node_pos.x -= depth_width_accumulator;
			node_pos.x -= this_depth_width;
			int64_t obj_id = grouped_nodes[i][j]->get_instance_id();
			node_pos.x += (this_depth_width - ImNodes::GetNodeDimensions(obj_id).x)*0.5f;

			node_pos.y = height_accumulator;
			node_pos.y -= this_depth_height * 0.5f;
			ImNodes::SetNodeEditorSpacePos(obj_id, node_pos);

			height_accumulator += ImNodes::GetNodeDimensions(obj_id).y + VERTICAL_MARGIN;
		}
		depth_width_accumulator += this_depth_width + HORIZONTAL_MARGIN;
	}
}
int EPASController::get_nodes_by_depth(Ref<EPASNode> p_node, Vector<Vector<Ref<EPASNode>>> &r_nodes_by_depth, int p_current_depth) const {
	r_nodes_by_depth.ptrw()[p_current_depth].push_back(p_node);
	p_current_depth++;

	int highest_depth = p_current_depth;

	for (int i = 0; i < p_node->get_input_count(); i++) {
		Ref<EPASNode> child_node = p_node->get_input(i);
		if (!child_node.is_valid()) {
			continue;
		}
		highest_depth = MAX(highest_depth, get_nodes_by_depth(child_node, r_nodes_by_depth, p_current_depth));
	}

	return highest_depth;
}
#endif

Ref<EPASPose> EPASController::get_base_pose() {
	Skeleton3D *skel = get_skeleton();
	if (!skel) {
		// Skeleton is gone, which means our pose is going to get invalidated
		base_pose_cache = Ref<EPASPose>();
		return base_pose_cache;
	}
	if (!base_pose_cache.is_valid() || base_pose_dirty || skel->get_version() != skeleton_version) {
		base_pose_cache = Ref<EPASPose>(memnew(EPASPose));

		for (int i = 0; i < skel->get_bone_count(); i++) {
			StringName bone_name = skel->get_bone_name(i);
			base_pose_cache->create_bone(bone_name);
			Transform3D rest = skel->get_bone_rest(i);
			base_pose_cache->set_bone_position(bone_name, rest.origin);
			base_pose_cache->set_bone_rotation(bone_name, rest.get_basis().get_rotation_quaternion());
			base_pose_cache->set_bone_scale(bone_name, rest.get_basis().get_scale());
		}
		skeleton_version = skel->get_version();
		base_pose_dirty = false;
	}
	return base_pose_cache;
}

void EPASController::advance(float p_amount) {
	Ref<EPASPose> base_pose = get_base_pose();
	if (!base_pose.is_valid()) {
		// Base pose is empty, which means there must not be a skeleton available
		return;
	}
	if (!output_pose.is_valid()) {
		output_pose.instantiate();
	}
	output_pose->clear();
	root->process_node(base_pose, output_pose, p_amount);

	Skeleton3D *skel = get_skeleton();
	// By this point skeleton should exist, if it doesn't something must have gone wrong
	ERR_FAIL_COND_MSG(!skel, "EPASController: skeleton is missing, major malfunction.");

	for (int i = 0; i < skel->get_bone_count(); i++) {
		StringName bone_name = skel->get_bone_name(i);
		if (ignored_bones.has(bone_name)) {
			continue;
		}
		if (!output_pose->has_bone(bone_name)) {
			skel->set_bone_pose_position(i, base_pose->get_bone_position(bone_name));
			skel->set_bone_pose_rotation(i, base_pose->get_bone_rotation(bone_name));
			skel->set_bone_pose_scale(i, base_pose->get_bone_scale(bone_name));
		} else {
			skel->set_bone_pose_position(i, output_pose->get_bone_position(bone_name, base_pose));
			skel->set_bone_pose_rotation(i, output_pose->get_bone_rotation(bone_name, base_pose));
			skel->set_bone_pose_scale(i, output_pose->get_bone_scale(bone_name, base_pose));
		}
	}

#ifdef DEBUG_ENABLED
	if (debug_enable_skeleton_vis) {
		_debug_update_skeleton_vis();
	} else if (debug_skeleton_vis) {
		debug_skeleton_vis->hide();
	}
#endif
}

void EPASController::connect_node_to_root(Ref<EPASNode> p_from, StringName p_unique_name) {
	connect_node(p_from, root, p_unique_name, 0);
}

void EPASController::connect_node(Ref<EPASNode> p_from, Ref<EPASNode> p_to, StringName p_unique_name, int p_input) {
	ERR_FAIL_COND_MSG(!nodes.has(p_to), "Trying to connect to a node that is not part of this controller");
	ERR_FAIL_COND_MSG(nodes.has(p_from), "Trying to connect a node that is already part of this controller");
	ERR_FAIL_COND_MSG(node_name_map.has(p_unique_name), "Trying to use a node unique name that is already used in this controller");
	p_to->connect_to_input(p_input, p_from);
	nodes.push_back(p_from);
	node_name_map.insert(p_unique_name, p_from);
	p_from->set_epas_controller(this);
#ifdef DEBUG_ENABLED
	p_from->set_meta("epas_name", p_unique_name);
#endif
}

Ref<EPASNode> EPASController::get_epas_node(const StringName &p_node_name) const {
	ERR_FAIL_COND_V_MSG(!node_name_map.has(p_node_name), Ref<EPASNode>(), vformat("EPAS node %s not found", p_node_name));
	return node_name_map[p_node_name];
}

Ref<EPASPose> EPASController::get_output_pose() const {
	return output_pose;
}

void EPASController::ignore_bones(const TypedArray<StringName> &p_bone_names) {
	ignored_bones.append_array(p_bone_names);
}

void EPASController::clear_ignored_bones() {
	ignored_bones.clear();
}

void EPASController::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_playback_process_mode", "playback_process_mode"), &EPASController::set_playback_process_mode);
	ClassDB::bind_method(D_METHOD("get_playback_process_mode"), &EPASController::get_playback_process_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "playback_process_mode", PROPERTY_HINT_ENUM, "Idle,Physics,Manual"), "set_playback_process_mode", "get_playback_process_mode");

	ClassDB::bind_method(D_METHOD("set_skeleton_path", "skeleton_path"), &EPASController::set_skeleton_path);
	ClassDB::bind_method(D_METHOD("get_skeleton_path"), &EPASController::get_skeleton_path);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "skeleton_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Skeleton3D"), "set_skeleton_path", "get_skeleton_path");

	ClassDB::bind_method(D_METHOD("advance", "amount"), &EPASController::advance);
	ClassDB::bind_method(D_METHOD("connect_node_to_root", "from", "unique_name"), &EPASController::connect_node_to_root);
	ClassDB::bind_method(D_METHOD("connect_node", "from", "to", "unique_name", "input"), &EPASController::connect_node);
	ClassDB::bind_method(D_METHOD("get_epas_node", "node_name"), &EPASController::get_epas_node);
	ClassDB::bind_method(D_METHOD("get_base_pose"), &EPASController::get_base_pose);

	BIND_ENUM_CONSTANT(IDLE);
	BIND_ENUM_CONSTANT(PHYSICS_PROCESS);
	BIND_ENUM_CONSTANT(MANUAL);
}

void EPASController::_update_skeleton_node_cache() {
	skeleton_node_cache = ObjectID();

	if (has_node(skeleton_path)) {
		Node *node = get_node(skeleton_path);
		ERR_FAIL_COND_MSG(!node, "Cannot update actor graphics cache: Node cannot be found!");

		// Ensure its a Node3D
		Skeleton3D *nd = Object::cast_to<Skeleton3D>(node);
		ERR_FAIL_COND_MSG(!nd, "Cannot update EPAS skeleton node cache: NodePath does not point to a Skeleton3D node!");

		skeleton_node_cache = nd->get_instance_id();
	}
}

void EPASController::set_skeleton_path(const NodePath &p_skeleton_path) {
	skeleton_path = p_skeleton_path;
	base_pose_dirty = true;
	_update_skeleton_node_cache();
}

NodePath EPASController::get_skeleton_path() const {
	return skeleton_path;
}

EPASController::PlaybackProcessMode EPASController::get_playback_process_mode() const {
	return playback_process_mode;
}

Skeleton3D *EPASController::get_skeleton() {
	if (skeleton_node_cache.is_valid()) {
		return Object::cast_to<Skeleton3D>(ObjectDB::get_instance(skeleton_node_cache));
	} else {
		_update_skeleton_node_cache();
		if (skeleton_node_cache.is_valid()) {
			return Object::cast_to<Skeleton3D>(ObjectDB::get_instance(skeleton_node_cache));
		}
	}

	return nullptr;
}

EPASController::EPASController() {
	root = Ref<EPASRootNode>(memnew(EPASRootNode));
	nodes.push_back(root);
	node_name_map.insert("Output", root);
	print_line("ROOT CREATE");
#ifdef DEBUG_ENABLED
	set_process_internal(true);
	root->set_meta("epas_name", "Output");
	hip_plot_lines_x.resize_zeroed(90);
	hip_plot_lines_y.resize_zeroed(90);
#endif
}

EPASController::~EPASController() {
	node_name_map.clear();
	nodes.clear();
	print_line("CONTROLLER OUT!");
}
