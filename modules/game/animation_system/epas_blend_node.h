#ifndef EPAS_BLEND_NODE_H
#define EPAS_BLEND_NODE_H
#include "epas_node.h"

class EPASBlendNode : public EPASNode {
	GDCLASS(EPASBlendNode, EPASNode);

private:
	float blend_amount = 0.0f;
	TypedArray<StringName> bone_filter;

protected:
	static void _bind_methods();

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
	virtual Control* _debug_node_create() override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	void set_blend_amount(float p_blend_amount);
	float get_blend_amount() const;
	EPASBlendNode();

	TypedArray<StringName> get_bone_filter() const;
	void set_bone_filter(const TypedArray<StringName> &p_bone_filter);
};
#endif // EPAS_BLEND_NODE_H