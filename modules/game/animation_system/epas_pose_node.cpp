#include "epas_pose_node.h"

#ifdef DEBUG_ENABLED
#include "modules/imgui/godot_imgui.h"
#endif

void EPASPoseNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_pose"), &EPASPoseNode::get_pose);
	ClassDB::bind_method(D_METHOD("set_pose", "pose"), &EPASPoseNode::set_pose);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "pose", PROPERTY_HINT_RESOURCE_TYPE, "EPASPose"), "set_pose", "get_pose");
}

void EPASPoseNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	// This should be ok for pose nodes, since p_target_pose should be empty...
	if (!pose.is_valid()) {
		return;
	}
	p_target_pose->reserve(pose->get_bone_count());
	for (KeyValue<String, EPASPose::BoneData *> kv : pose->get_bone_map()) {
		EPASPose::BoneData *target = p_target_pose->get_bone_data(kv.key);
		EPASPose::BoneData *source = kv.value;
		if (!target) {
			target = p_target_pose->create_bone(kv.key);
		}
		if (source->has_position) {
			target->has_position = true;
			target->position = source->position;
		}
		if (source->has_rotation) {
			target->has_rotation = true;
			target->rotation = source->rotation;
		}
		if (source->has_scale) {
			target->has_scale = true;
			target->scale = source->scale;
		}
	}
}

void EPASPoseNode::set_pose(const Ref<EPASPose> &p_pose) {
	pose = p_pose;
}

#ifdef DEBUG_ENABLED
void EPASPoseNode::_debug_node_draw() const {
	if (!pose.is_valid()) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0, 1.0, 0.0, 1.0));
		ImGui::TextUnformatted("!!WARNING!!");
		ImGui::TextUnformatted("No pose");
		ImGui::PopStyleColor();
	}
}
#endif

Ref<EPASPose> EPASPoseNode::get_pose() const {
	return pose;
}

EPASPoseNode::EPASPoseNode() :
		EPASNode(get_input_count()) {
}