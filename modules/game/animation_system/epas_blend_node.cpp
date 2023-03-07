#include "epas_blend_node.h"

void EPASBlendNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_blend_amount", "amount"), &EPASBlendNode::set_blend_amount);
	ClassDB::bind_method(D_METHOD("get_blend_amount"), &EPASBlendNode::get_blend_amount);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_amount", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_blend_amount", "get_blend_amount");
}

void EPASBlendNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	if (blend_amount == 0.0f) {
		// Fast path for 0 addition
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
		return;
	} else {
		// Ideas for second pose
		// keep our own epaspose object that we fill up once and then we just modify
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
		Ref<EPASPose> second_pose = memnew(EPASPose);
		process_input_pose(1, p_base_pose, second_pose, p_delta);

		p_target_pose->blend(second_pose, p_base_pose, p_target_pose, blend_amount);
	}
}

void EPASBlendNode::set_blend_amount(float p_blend_amount) {
	p_blend_amount = p_blend_amount;
}

float EPASBlendNode::get_blend_amount() const {
	return blend_amount;
}

#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
void EPASBlendNode::_debug_node_draw() const {
	ImGui::PushItemWidth(100.0f);
	ImGui::SliderFloat("Amount", const_cast<float *>(&blend_amount), 0.0f, 1.0f);
	ImGui::PopItemWidth();
};
#endif

EPASBlendNode::EPASBlendNode() {
	_set_input_count(2);
}