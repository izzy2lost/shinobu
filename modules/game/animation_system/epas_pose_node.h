#ifndef EPAS_POSE_NODE_H
#define EPAS_POSE_NODE_H
#include "epas_node.h"
#include "epas_pose.h"

class EPASPoseNode : public EPASNode {
	GDCLASS(EPASPoseNode, EPASNode);
	Ref<EPASPose> pose;

protected:
	static void _bind_methods();

public:
	void set_pose(const Ref<EPASPose> &p_pose);
	Ref<EPASPose> get_pose() const;
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
};
#endif // EPASPoseNode