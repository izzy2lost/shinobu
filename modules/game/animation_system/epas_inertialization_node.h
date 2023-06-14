#ifndef EPAS_INERTIALIZATION_NODE_H
#define EPAS_INERTIALIZATION_NODE_H

#include "../inertialization.h"
#include "epas_node.h"

class EPASInertializationNode : public EPASNode {
	GDCLASS(EPASInertializationNode, EPASNode);
	float desired_blend_time = 0.0f;
	Ref<EPASPose> last_frame_pose;
	Ref<EPASPose> last_last_frame_pose;
	bool inertialization_queued = false;
	Ref<EPASPoseInertializer> pose_inertializer;
	TypedArray<StringName> bone_filter;

protected:
	void static _bind_methods();
	void start_inertialization(const Ref<EPASPose> &p_base_pose, const Ref<EPASPose> &p_current_pose, float p_delta);
	void process_input_pose_inertialized(int p_input, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> &p_target_pose, float p_delta);

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	void inertialize(float p_transition_duration = 0.25f, TypedArray<StringName> p_bone_filter = TypedArray<StringName>());
	void flush_inertialization();
	bool is_inertializing() const;
	EPASInertializationNode();
};

#endif // EPAS_INERTIALIZATION_NODE_H
