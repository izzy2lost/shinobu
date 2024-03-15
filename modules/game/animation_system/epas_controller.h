/**************************************************************************/
/*  epas_controller.h                                                     */
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

#ifndef EPAS_CONTROLLER_H
#define EPAS_CONTROLLER_H

#include "epas_node.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/main/node.h"
#include "scene/resources/audio_stream_polyphonic.h"

class AudioStreamPlayer3D;

class EPASRootNode : public EPASNode {
public:
	void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override {
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
	}

	EPASRootNode() {
		_set_input_count(1);
	}
};

class EPASController : public Node {
	GDCLASS(EPASController, Node);

	AudioStreamPlayer3D *audio_player = nullptr;

#ifdef DEBUG_ENABLED
	Vector<float> hip_plot_lines_y;
	Vector<float> hip_plot_lines_x;
	float plot_t = 0.0f;
#endif

public:
	enum PlaybackProcessMode {
		IDLE,
		PHYSICS_PROCESS,
		MANUAL,
	};

private:
#ifdef DEBUG_ENABLED
	bool debug_node_viewer_enabled = false;
#endif
	PlaybackProcessMode playback_process_mode = PlaybackProcessMode::IDLE;
	void _update_process_mode();
	ObjectID skeleton_node_cache;
	NodePath skeleton_path;
	void _update_skeleton_node_cache();

	Ref<EPASRootNode> root;
	HashMap<StringName, Ref<EPASNode>> node_name_map;
	Vector<Ref<EPASNode>> nodes;

	Ref<EPASPose> output_pose;

	bool base_pose_dirty = true;
	uint64_t skeleton_version = 0;
	Ref<EPASPose> base_pose_cache;
	void _update_base_pose();

#ifdef DEBUG_ENABLED
	int debug_draw_accumulator = 0;
	int debug_draw_attrib_accumulator = 0;
	int debug_draw_link_accumulator = 0;
	bool debug_enable_skeleton_vis = false;
	Vector<Vector3> skel_dbg_vertex_array;
	void _debug_draw_node(Ref<EPASNode> p_node, int *p_output_attrib_id);
	MeshInstance3D *debug_skeleton_vis = nullptr;
	int _get_skeleton_line_count(Skeleton3D *p_skel);
	void _debug_update_skeleton_vis();
	void _arrange_nodes();
	int get_nodes_by_depth(Ref<EPASNode> p_node, Vector<Vector<Ref<EPASNode>>> &r_nodes_by_depth, int p_current_depth = 0) const;
#endif
	TypedArray<StringName> ignored_bones;

protected:
	void _notification(int p_what);
#ifdef DEBUG_ENABLED
	void _notification_debug(int p_what);
#endif
	static void _bind_methods();

public:
	Ref<AudioStreamPlaybackPolyphonic> get_audio_stream_playback() const;
	Skeleton3D *get_skeleton();
	Ref<EPASPose> get_base_pose();
	void set_playback_process_mode(PlaybackProcessMode p_playback_process_mode);
	PlaybackProcessMode get_playback_process_mode() const;
	void set_skeleton_path(const NodePath &p_skeleton_path);
	NodePath get_skeleton_path() const;
	void advance(float p_amount);
	void connect_node_to_root(Ref<EPASNode> p_from, StringName p_unique_name);
	void connect_node(Ref<EPASNode> p_from, Ref<EPASNode> p_to, StringName p_unique_name, int p_input);
	Ref<EPASNode> get_epas_node(const StringName &p_node_name) const;

	Ref<EPASPose> get_output_pose() const;
	void ignore_bones(const TypedArray<StringName> &p_bone_names);
	void clear_ignored_bones();

	EPASController();
	~EPASController();
	friend class EPASNode;
};

VARIANT_ENUM_CAST(EPASController::PlaybackProcessMode);

#endif // EPAS_CONTROLLER_H