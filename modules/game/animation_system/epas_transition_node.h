#ifndef EPAS_TRANSITION_NODE_H
#define EPAS_TRANSITION_NODE_H

#include "epas_node.h"

class EPASTransitionNode : public EPASNode {
	GDCLASS(EPASTransitionNode, EPASNode);

private:
	int current_input = 0;

protected:
	static void _bind_methods();

public:
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	void transition_to(int p_current_input);
	void set_transition_count(int p_transition_count);
	int get_transition_count() const;

#ifdef DEBUG_ENABLED
	void _debug_node_draw() const override;
#endif
};

#endif // EPAS_TRANSITION_NODE_H
