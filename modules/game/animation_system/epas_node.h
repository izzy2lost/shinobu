#ifndef EPAS_NODE_H
#define EPAS_NODE_H

#include "epas_pose.h"

class EPASController;

class EPASNode : public RefCounted {
	GDCLASS(EPASNode, RefCounted);

	Vector<Ref<EPASNode>> children;
	EPASController *epas_controller;

private:
protected:
	void set_epas_controller(EPASController *p_epas_controller);
	EPASController *get_epas_controller() const;
	static void _bind_methods();
	Skeleton3D *get_skeleton() const;

public:
	virtual int get_input_count() const;
	virtual void connect_to_input(int p_input, Ref<EPASNode> p_node);
	void process_input_pose(int p_child, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta);
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta){};
	Ref<EPASNode> get_input(int p_input) const;
	EPASNode(int p_input_count);
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const {};
#endif
	virtual ~EPASNode(){};
	friend class EPASController;
};

#endif // EPAS_NODE_H