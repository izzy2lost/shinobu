#ifndef EPAS_ADD_NODE_H
#define EPAS_ADD_NODE_H
#include "epas_node.h"

class EPASAddNode : public EPASNode {
	GDCLASS(EPASAddNode, EPASNode);

private:
	float add_amount = 0.0f;

protected:
	static void _bind_methods();

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	void set_add_amount(float p_add_amount);
	float get_add_amount() const;
	EPASAddNode();
};
#endif // EPAS_ADD_NODE_H