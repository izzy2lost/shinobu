#include "epas_add_node.h"
#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#endif

void EPASAddNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_add_amount", "amount"), &EPASAddNode::set_add_amount);
	ClassDB::bind_method(D_METHOD("get_add_amount"), &EPASAddNode::get_add_amount);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "add_amount", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_add_amount", "get_add_amount");
}

void EPASAddNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	if (add_amount == 0.0f) {
		// Fast path for 0 addition
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
		return;
	} else {
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
		Ref<EPASPose> second_pose = memnew(EPASPose);
		process_input_pose(1, p_base_pose, second_pose, p_delta);

		p_target_pose->add(second_pose, p_base_pose, p_target_pose, add_amount);
	}
}

void EPASAddNode::set_add_amount(float p_add_amount) {
	add_amount = p_add_amount;
}

float EPASAddNode::get_add_amount() const {
	return add_amount;
}
#ifdef DEBUG_ENABLED
void EPASAddNode::_debug_node_draw() const {
	ImGui::PushItemWidth(100.0f);
	ImGui::SliderFloat("Amount", const_cast<float *>(&add_amount), 0.0f, 1.0f);
	ImGui::PopItemWidth();
};
#endif

EPASAddNode::EPASAddNode() {
	_set_input_count(2);
}