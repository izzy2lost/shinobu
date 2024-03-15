/**************************************************************************/
/*  epas_blend_node.cpp                                                   */
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

#include "epas_blend_node.h"
#include "scene/gui/box_container.h"
#include "scene/gui/slider.h"

void EPASBlendNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_blend_amount", "amount"), &EPASBlendNode::set_blend_amount);
	ClassDB::bind_method(D_METHOD("get_blend_amount"), &EPASBlendNode::get_blend_amount);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "blend_amount", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_blend_amount", "get_blend_amount");

	ClassDB::bind_method(D_METHOD("set_bone_filter", "bone_filter"), &EPASBlendNode::set_bone_filter);
	ClassDB::bind_method(D_METHOD("get_bone_filter"), &EPASBlendNode::get_bone_filter);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "bone_filter", PROPERTY_HINT_ARRAY_TYPE, "StringName"), "set_bone_filter", "get_bone_filter");
}

void EPASBlendNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	process_input_pose(0, p_base_pose, p_target_pose, p_delta);
	Ref<EPASPose> second_pose = memnew(EPASPose);
	// The reason we process the second node even if the blend is 0 is to keep them in sync in case that's our intention
	process_input_pose(1, p_base_pose, second_pose, p_delta);

	p_target_pose->blend(second_pose, p_base_pose, p_target_pose, blend_amount, bone_filter);
}

void EPASBlendNode::set_blend_amount(float p_blend_amount) {
	blend_amount = p_blend_amount;
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
TypedArray<StringName> EPASBlendNode::get_bone_filter() const {
	return bone_filter;
}
void EPASBlendNode::set_bone_filter(const TypedArray<StringName> &p_bone_filter) {
	bone_filter = p_bone_filter;
}
