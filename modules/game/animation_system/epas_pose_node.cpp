/**************************************************************************/
/*  epas_pose_node.cpp                                                    */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

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
