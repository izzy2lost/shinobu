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
				first_pose_d->position = first_pos.lerp(second_bone_d->position, blend_amount);
			}
			if (second_bone_d->has_rotation) {
				Quaternion first_rotation = first_pose_d->get_rotation(base_pose_d);
				first_pose_d->has_rotation = true;
				first_pose_d->rotation = first_rotation.slerp(second_bone_d->rotation, blend_amount);
			}
			if (second_bone_d->has_scale) {
				Vector3 first_scale = first_pose_d->get_scale(base_pose_d);
				first_pose_d->has_scale = true;
				first_pose_d->scale = first_scale.lerp(second_bone_d->scale, blend_amount);
			}
		}
	}
}

int EPASBlendNode::get_input_count() const {
	return 2;
}

void EPASBlendNode::set_blend_amount(float p_blend_amount) {
	p_blend_amount = p_blend_amount;
}

float EPASBlendNode::get_blend_amount() const {
	return blend_amount;
}

EPASBlendNode::EPASBlendNode() :
		EPASNode(get_input_count()) {
}