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
		// Ideas for second pose
		// keep our own epaspose object that we fill up once and then we just modify
		process_input_pose(0, p_base_pose, p_target_pose, p_delta);
		Ref<EPASPose> second_pose = memnew(EPASPose);
		process_input_pose(1, p_base_pose, second_pose, p_delta);

		for (const KeyValue<String, EPASPose::BoneData *> &kv : second_pose->get_bone_map()) {
			// this is the output bonedata
			EPASPose::BoneData *first_pose_d = p_target_pose->get_bone_data(kv.value->bone_name);
			EPASPose::BoneData *second_bone_d = kv.value;
			EPASPose::BoneData *base_pose_d = p_base_pose->get_bone_data(kv.key);

			if (!base_pose_d) {
				// Bone doesn't exist in skeleton, skip.
				continue;
			}

			if (!first_pose_d) {
				first_pose_d = p_target_pose->create_bone(kv.key);
				*first_pose_d = *base_pose_d;
			}

			if (second_bone_d->has_position) {
				Vector3 first_pos = first_pose_d->get_position(base_pose_d);
				first_pose_d->has_position = true;
				first_pose_d->position = first_pos.lerp(first_pos + second_bone_d->position, add_amount);
			}
			if (second_bone_d->has_rotation) {
				Quaternion first_rotation = first_pose_d->get_rotation(base_pose_d);
				first_pose_d->has_rotation = true;
				first_pose_d->rotation = first_rotation.slerp(second_bone_d->rotation * first_rotation, add_amount);
			}
			if (second_bone_d->has_scale) {
				Vector3 first_scale = first_pose_d->get_scale(base_pose_d);
				first_pose_d->has_scale = true;
				first_pose_d->scale = first_scale.lerp(first_scale + second_bone_d->scale, add_amount);
			}
		}
	}
}

int EPASAddNode::get_input_count() const {
	return 2;
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
EPASAddNode::EPASAddNode() :
		EPASNode(get_input_count()) {
}