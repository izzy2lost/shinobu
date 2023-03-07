#include "epas_node.h"
#include "core/error/error_macros.h"
#include "epas_controller.h"
#include "scene/3d/skeleton_3d.h"

void EPASNode::set_epas_controller(EPASController *p_epas_controller) {
	epas_controller = p_epas_controller;
}

EPASController *EPASNode::get_epas_controller() const {
	ERR_FAIL_COND_V_MSG(!epas_controller, nullptr, "Bug?");
	return epas_controller;
}

void EPASNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_input_count"), &EPASNode::get_input_count);
	ClassDB::bind_method(D_METHOD("connect_to_input", "input", "node"), &EPASNode::connect_to_input);
	ClassDB::bind_method(D_METHOD("process", "base_pose", "target_pose", "delta"), &EPASNode::process_node);
	ClassDB::bind_method(D_METHOD("process_input_pose", "input", "base_pose", "target_pose", "delta"), &EPASNode::process_input_pose);
}

Skeleton3D *EPASNode::get_skeleton() const {
	ERR_FAIL_COND_V(epas_controller == nullptr, nullptr);
	Skeleton3D *skel = epas_controller->get_skeleton();
	return skel;
}

void EPASNode::_set_input_count(int p_count) {
	children.resize_zeroed(p_count);
}

int EPASNode::get_input_count() const {
	return children.size();
}

void EPASNode::process_input_pose(int p_child, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	ERR_FAIL_INDEX_MSG(p_child, get_input_count(), vformat("Invalid child number: %d", p_child));
	if (children[p_child].is_valid()) {
		Ref<EPASNode> child = children[p_child];
		child->process_node(p_base_pose, p_target_pose, p_delta);
	}
}

void EPASNode::connect_to_input(int p_input, Ref<EPASNode> p_node) {
	ERR_FAIL_INDEX_MSG(p_input, get_input_count(), vformat("Invalid input number: %d", p_input));
	ERR_FAIL_COND_MSG(children[p_input].is_valid(), "This input is already connected");
	children.set(p_input, p_node);
}

Ref<EPASNode> EPASNode::get_input(int p_input) const {
	ERR_FAIL_INDEX_V_MSG(p_input, get_input_count(), Ref<EPASNode>(), vformat("Invalid input number: %d", p_input));
	return children[p_input];
}
