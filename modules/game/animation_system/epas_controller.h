#ifndef EPAS_CONTROLLER_H
#define EPAS_CONTROLLER_H

#include "epas_node.h"
#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/main/node.h"

class EPASRootNode : public EPASNode {
public:
	virtual int get_input_count() const override {
		return 1;
	}
	void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override {
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
	}

	EPASRootNode() :
			EPASNode(get_input_count()) {}
};

class EPASController : public Node {
	GDCLASS(EPASController, Node);

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
	Skeleton3D *get_skeleton();

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
#endif

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	Ref<EPASPose> get_base_pose();
	void set_playback_process_mode(PlaybackProcessMode p_playback_process_mode);
	PlaybackProcessMode get_playback_process_mode() const;
	void set_skeleton_path(const NodePath &p_skeleton_path);
	NodePath get_skeleton_path() const;
	void advance(float p_amount);
	void connect_node_to_root(Ref<EPASNode> p_from, StringName p_unique_name);
	void connect_node(Ref<EPASNode> p_from, Ref<EPASNode> p_to, StringName p_unique_name, int p_input);

	Ref<EPASPose> get_output_pose() const;

	EPASController();
	~EPASController();
	friend class EPASNode;
};

VARIANT_ENUM_CAST(EPASController::PlaybackProcessMode);

#endif // EPAS_CONTROLLER_H