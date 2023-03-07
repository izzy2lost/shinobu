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
	for (int i = 0; i < pose->get_bone_count(); i++) {
		StringName bone_name = pose->get_bone_name(i);
		if (!p_target_pose->has_bone(bone_name)) {
			p_target_pose->create_bone(bone_name);
		}
		if (pose->get_bone_has_position(bone_name)) {
			p_target_pose->set_bone_position(bone_name, pose->get_bone_position(bone_name));
		}
		if (pose->get_bone_has_rotation(bone_name)) {
			p_target_pose->set_bone_rotation(bone_name, pose->get_bone_rotation(bone_name));
		}
		if (pose->get_bone_has_scale(bone_name)) {
			p_target_pose->set_bone_scale(bone_name, pose->get_bone_scale(bone_name));
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
