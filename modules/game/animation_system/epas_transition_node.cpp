#include "epas_transition_node.h"

void EPASTransitionNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_transition_count"), &EPASTransitionNode::get_transition_count);
	ClassDB::bind_method(D_METHOD("set_transition_count", "input_count"), &EPASTransitionNode::set_transition_count);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "transition_count", PROPERTY_HINT_RANGE, "0,20"), "set_transition_count", "get_transition_count");

	ClassDB::bind_method(D_METHOD("transition_to", "target"), &EPASTransitionNode::transition_to);

	ADD_SIGNAL(MethodInfo("transitioned"));
}

void EPASTransitionNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	process_input_pose(current_input, p_base_pose, p_target_pose, p_delta);
}

void EPASTransitionNode::transition_to(int p_current_input) {
	ERR_FAIL_INDEX_MSG(p_current_input, get_input_count(), "Invalid input");
	if (p_current_input != current_input) {
		current_input = p_current_input;
		emit_signal(SNAME("transitioned"));
	}
}

int EPASTransitionNode::get_transition_count() const {
	return get_input_count();
}

void EPASTransitionNode::set_transition_count(int p_transition_count) {
	_set_input_count(p_transition_count);
}

#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
void EPASTransitionNode::_debug_node_draw() const {
	ImGui::Text("Transition: %d", current_input);
}
#endif