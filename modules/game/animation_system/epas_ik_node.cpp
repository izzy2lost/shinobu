#include "epas_ik_node.h"
#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/object/class_db.h"
#include "core/object/object.h"
#include "epas_node.h"
#include "modules/game/utils.h"

void EPASIKNode::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_ik_influence"), &EPASIKNode::get_ik_influence);
	ClassDB::bind_method(D_METHOD("set_ik_influence", "ik_influence"), &EPASIKNode::set_ik_influence);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ik_influence"), "set_ik_influence", "get_ik_influence");

	ClassDB::bind_method(D_METHOD("get_use_magnet"), &EPASIKNode::get_use_magnet);
	ClassDB::bind_method(D_METHOD("set_use_magnet", "use_magnet"), &EPASIKNode::set_use_magnet);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_magnet"), "set_use_magnet", "get_use_magnet");

	ClassDB::bind_method(D_METHOD("get_target_position"), &EPASIKNode::get_target_position);
	ClassDB::bind_method(D_METHOD("set_target_position", "target_position"), &EPASIKNode::set_target_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target_position"), "set_target_position", "get_target_position");

	ClassDB::bind_method(D_METHOD("get_magnet_position"), &EPASIKNode::get_magnet_position);
	ClassDB::bind_method(D_METHOD("set_magnet_position", "magnet_position"), &EPASIKNode::set_magnet_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "magnet_position"), "set_magnet_position", "get_magnet_position");

	ClassDB::bind_method(D_METHOD("get_ik_end"), &EPASIKNode::get_ik_end);
	ClassDB::bind_method(D_METHOD("set_ik_end", "ik_end"), &EPASIKNode::set_ik_end);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "ik_end"), "set_ik_end", "get_ik_end");
}

int EPASIKNode::get_input_count() const {
	return 1;
}

void EPASIKNode::process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) {
	Skeleton3D *skel = get_skeleton();
	process_input_pose(0, p_base_pose, p_target_pose, p_delta);

	if (!skel || ik_influence == 0.0f) {
		return;
	}

	int c_bone_idx = skel->find_bone(ik_end);
	int b_bone_idx = skel->get_bone_parent(c_bone_idx);
	int a_bone_idx = skel->get_bone_parent(b_bone_idx);

	if (c_bone_idx == -1 || b_bone_idx == -1 || a_bone_idx == -1) {
		return;
	}

	String a_bone_name = skel->get_bone_name(a_bone_idx);
	String b_bone_name = skel->get_bone_name(b_bone_idx);
	String c_bone_name = ik_end;

	Transform3D a_global_trf = p_target_pose->calculate_bone_global_transform(a_bone_name, skel, p_base_pose);
	Transform3D b_global_trf = p_target_pose->calculate_bone_global_transform(b_bone_name, skel, p_base_pose);
	Transform3D c_global_trf = p_target_pose->calculate_bone_global_transform(c_bone_name, skel, p_base_pose);

	Quaternion a_global_rot = a_global_trf.basis.get_rotation_quaternion();
	Quaternion b_global_rot = b_global_trf.basis.get_rotation_quaternion();

	Quaternion a_local_rot = p_target_pose->get_bone_rotation(a_bone_name, p_base_pose);
	Quaternion b_local_rot = p_target_pose->get_bone_rotation(b_bone_name, p_base_pose);

	HBUtils::two_joint_ik(
			a_global_trf.origin,
			b_global_trf.origin,
			c_global_trf.origin,
			target_position,
			0.01,
			a_global_rot,
			b_global_rot,
			a_local_rot,
			b_local_rot,
			use_magnet,
			skel->to_local(magnet_position).normalized());

	if (!p_target_pose->has_bone(a_bone_name)) {
		p_target_pose->create_bone(a_bone_name);
	}
	if (!p_target_pose->has_bone(b_bone_name)) {
		p_target_pose->create_bone(b_bone_name);
	}
	if (!p_target_pose->has_bone(c_bone_name)) {
		p_target_pose->create_bone(c_bone_name);
	}

	Quaternion prev_a_local_rot = p_target_pose->get_bone_rotation(a_bone_name, p_base_pose);
	Quaternion prev_b_local_rot = p_target_pose->get_bone_rotation(b_bone_name, p_base_pose);

	p_target_pose->set_bone_rotation(a_bone_name, prev_a_local_rot.slerp(a_local_rot, ik_influence));
	p_target_pose->set_bone_rotation(b_bone_name, prev_b_local_rot.slerp(b_local_rot, ik_influence));

	// Apply tip rotation
	//b_global_trf = p_target_pose->calculate_bone_global_transform(b_bone_name, skel, p_base_pose);
	//p_target_pose->set_bone_rotation(c_bone_name, b_global_trf.basis.get_rotation_quaternion().inverse() * c_global_rot);
}

float EPASIKNode::get_ik_influence() const {
	return ik_influence;
}

void EPASIKNode::set_ik_influence(float p_ik_influence) {
	ik_influence = p_ik_influence;
}

Vector3 EPASIKNode::get_target_position() const {
	return target_position;
}

void EPASIKNode::set_target_position(const Vector3 &p_target_position) {
	target_position = p_target_position;
}

bool EPASIKNode::get_use_magnet() const {
	return use_magnet;
}

void EPASIKNode::set_use_magnet(bool p_use_magnet) {
	use_magnet = p_use_magnet;
}

Vector3 EPASIKNode::get_magnet_position() const {
	return magnet_position;
}

void EPASIKNode::set_magnet_position(const Vector3 &p_magnet_position) {
	magnet_position = p_magnet_position;
}

String EPASIKNode::get_ik_end() const {
	return ik_end;
}

void EPASIKNode::set_ik_end(const String &p_ik_end) {
	ik_end = p_ik_end;
}

EPASIKNode::EPASIKNode() :
		EPASNode(get_input_count()) {}
