/**************************************************************************/
/*  epas_pose.cpp                                                         */
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

#include "epas_pose.h"
#include "core/error/error_macros.h"
#include "core/math/quaternion.h"
#include "core/math/transform_2d.h"
#include "core/math/transform_3d.h"

void EPASPose::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_bone_position", "bone_name", "position"), &EPASPose::set_bone_position);
	ClassDB::bind_method(D_METHOD("set_bone_rotation", "bone_name", "rotation"), &EPASPose::set_bone_rotation);
	ClassDB::bind_method(D_METHOD("set_bone_scale", "bone_name", "scale"), &EPASPose::set_bone_scale);
	ClassDB::bind_method(D_METHOD("get_bone_position", "bone_name", "base_pose"), &EPASPose::get_bone_position, DEFVAL(Ref<EPASPose>()));
	ClassDB::bind_method(D_METHOD("get_bone_rotation", "bone_name", "base_pose"), &EPASPose::get_bone_rotation, DEFVAL(Ref<EPASPose>()));
	ClassDB::bind_method(D_METHOD("get_bone_scale", "bone_name", "base_pose"), &EPASPose::get_bone_scale, DEFVAL(Ref<EPASPose>()));
	ClassDB::bind_method(D_METHOD("get_bone_count"), &EPASPose::get_bone_count);
	ClassDB::bind_method(D_METHOD("get_bone_name", "bone_idx"), &EPASPose::get_bone_name);
	ClassDB::bind_method(D_METHOD("has_bone", "bone_name"), &EPASPose::has_bone);
	ClassDB::bind_method(D_METHOD("create_bone", "bone_name"), &EPASPose::create_bone_gd);
}

void EPASPose::_get_property_list(List<PropertyInfo> *p_list) const {
	p_list->push_back(PropertyInfo(Variant::DICTIONARY, "pose_data", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL));
}

bool EPASPose::_set(const StringName &p_name, const Variant &p_value) {
	if (p_name == SNAME("pose_data")) {
		Dictionary data = p_value;
		clear();
		for (int i = 0; i < data.size(); i++) {
			String key = data.get_key_at_index(i);
			if (!key.is_empty()) {
				BoneData *bone_data = memnew(BoneData(key));
				Dictionary data_dict = data.get(key, Dictionary());
				bone_data->has_position = data_dict.has("position");
				bone_data->has_rotation = data_dict.has("rotation");
				bone_data->has_scale = data_dict.has("scale");
				bone_data->bone_name = key;
				if (bone_data->has_position) {
					bone_data->position = data_dict["position"];
				}
				if (bone_data->has_rotation) {
					bone_data->rotation = data_dict["rotation"];
				}
				if (bone_data->has_scale) {
					bone_data->scale = data_dict["scale"];
				}
				bone_datas.insert(bone_data->bone_name, bone_data);
				bone_datas_v.push_back(bone_data);
			}
		}
		return true;
	}

	return false;
}

bool EPASPose::_get(const StringName &p_name, Variant &r_ret) const {
	if (p_name == SNAME("pose_data")) {
		Dictionary dic_out;
		for (const KeyValue<StringName, BoneData *> &kv : bone_datas) {
			Dictionary bone_data_dic;
			BoneData *bone_data = kv.value;
			if (bone_data->has_position) {
				bone_data_dic["position"] = bone_data->position;
			}
			if (bone_data->has_rotation) {
				bone_data_dic["rotation"] = bone_data->rotation;
			}
			if (bone_data->has_scale) {
				bone_data_dic["scale"] = bone_data->scale;
			}

			if (!bone_data_dic.is_empty()) {
				dic_out[bone_data->bone_name] = bone_data_dic;
			}
		}
		r_ret = dic_out;
		return true;
	}
	return false;
}

void EPASPose::clear() {
	for (KeyValue<StringName, BoneData *> kv : bone_datas) {
		memdelete(kv.value);
	}
	bone_datas_v.clear();
	bone_datas.clear();
}

int EPASPose::get_bone_count() const {
	return bone_datas.size();
}

EPASPose::BoneData *EPASPose::get_bone_data(const StringName &p_bone_name) const {
	EPASPose::BoneData *const *pp = bone_datas.getptr(p_bone_name);
	return pp != nullptr ? *pp : nullptr;
}

const HashMap<StringName, EPASPose::BoneData *> EPASPose::get_bone_map() const {
	return bone_datas;
}

void EPASPose::reserve(int p_size) {
	bone_datas.reserve(p_size);
}

Transform3D EPASPose::calculate_bone_global_transform(const StringName &p_bone_name, const Skeleton3D *p_skel, const Ref<EPASPose> p_base_pose) const {
	// Global in this context means relative to the skeleton
	ERR_FAIL_COND_V(p_skel == nullptr, Transform3D());

	int bone_idx = p_skel->find_bone(p_bone_name);

	Vector<String> p_path;
	p_path.push_back(p_bone_name);
	int parent = bone_idx;
	while (parent != -1) {
		parent = p_skel->get_bone_parent(parent);
		if (parent != -1) {
			p_path.push_back(p_skel->get_bone_name(parent));
		}
	}

	String root_name = p_path[p_path.size() - 1];
	Transform3D trf = get_bone_transform(root_name, p_base_pose);

	for (int i = p_path.size() - 2; i >= 0; i--) {
		Transform3D local_trf = get_bone_transform(p_path[i], p_base_pose);
		trf = trf * local_trf;
	}

	return trf;
}

void EPASPose::create_bone(const StringName &p_bone_name) {
	ERR_FAIL_COND_MSG(bone_datas.has(p_bone_name), vformat("Bone %s already exists", p_bone_name));
	ERR_FAIL_COND(String(p_bone_name).is_empty());
	BoneData *bd = memnew(BoneData(p_bone_name));
	bone_datas_v.push_back(bd);
	bone_datas.insert(p_bone_name, bd);
}

void EPASPose::create_bone_gd(const StringName &p_bone_name) {
	create_bone(p_bone_name);
}

void EPASPose::set_bone_position(const StringName &p_bone_name, const Vector3 &p_position) {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	ERR_FAIL_COND_MSG(!bd, vformat("Bone %s does not exist", p_bone_name));
	bd->has_position = true;
	bd->position = p_position;
}

Vector3 EPASPose::get_bone_position(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose) const {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	if (p_base_pose.is_valid()) {
		ERR_FAIL_COND_V_MSG(!p_base_pose->has_bone(p_bone_name), Vector3(), "Bone does not exist in base pose");
		if (!bd) {
			return p_base_pose->get_bone_position(p_bone_name);
		}
		return bd->get_position(p_base_pose->get_bone_data(p_bone_name));
	}
	ERR_FAIL_COND_V_MSG(!bd, Vector3(), vformat("Bone %s does not exist", p_bone_name));
	ERR_FAIL_COND_V_MSG(!bd->has_position, Vector3(), vformat("Bone %s doesn't have a position in this pose", p_bone_name));
	return bd->position;
}

void EPASPose::set_bone_has_position(const StringName &p_bone_name, bool p_has_position) {
	ERR_FAIL_COND_MSG(!has_bone(p_bone_name), vformat("Bone %s does not exist", p_bone_name));
	get_bone_data(p_bone_name)->has_position = p_has_position;
}

bool EPASPose::get_bone_has_position(const StringName &p_bone_name) const {
	ERR_FAIL_COND_V_MSG(!has_bone(p_bone_name), false, vformat("Bone %s does not exist", p_bone_name));
	return get_bone_data(p_bone_name)->has_position;
}

void EPASPose::set_bone_rotation(const StringName &p_bone_name, const Quaternion &p_rotation) {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	ERR_FAIL_COND_MSG(!bd, vformat("Bone %s does not exist", p_bone_name));
	bd->has_rotation = true;
	bd->rotation = p_rotation;
}

Quaternion EPASPose::get_bone_rotation(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose) const {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	if (p_base_pose.is_valid()) {
		ERR_FAIL_COND_V_MSG(!p_base_pose->has_bone(p_bone_name), Quaternion(), "Bone does not exist in base pose");
		if (!bd) {
			return p_base_pose->get_bone_rotation(p_bone_name);
		}
		return bd->get_rotation(p_base_pose->get_bone_data(p_bone_name));
	}
	ERR_FAIL_COND_V_MSG(!bd, Quaternion(), vformat("Bone %s does not exist", p_bone_name));
	ERR_FAIL_COND_V_MSG(!bd->has_rotation, Quaternion(), vformat("Bone %s doesn't have a rotation in this pose", p_bone_name));
	return bd->rotation;
}

void EPASPose::set_bone_has_rotation(const StringName &p_bone_name, bool p_has_rotation) {
	ERR_FAIL_COND_MSG(!has_bone(p_bone_name), vformat("Bone %s does not exist", p_bone_name));
	get_bone_data(p_bone_name)->has_rotation = p_has_rotation;
}

bool EPASPose::get_bone_has_rotation(const StringName &p_bone_name) const {
	ERR_FAIL_COND_V_MSG(!has_bone(p_bone_name), false, vformat("Bone %s does not exist", p_bone_name));
	return get_bone_data(p_bone_name)->has_rotation;
}

void EPASPose::set_bone_scale(const StringName &p_bone_name, const Vector3 &p_scale) {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	ERR_FAIL_COND_MSG(!bd, vformat("Bone %s does not exist", p_bone_name));
	bd->has_scale = true;
	bd->scale = p_scale;
}

Vector3 EPASPose::get_bone_scale(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose) const {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	if (p_base_pose.is_valid()) {
		ERR_FAIL_COND_V_MSG(!p_base_pose->has_bone(p_bone_name), Vector3(1.0f, 1.0f, 1.0f), "Bone does not exist in base pose");
		if (!bd) {
			return p_base_pose->get_bone_scale(p_bone_name);
		}
		return bd->get_scale(p_base_pose->get_bone_data(p_bone_name));
	}
	ERR_FAIL_COND_V_MSG(!bd, Vector3(), vformat("Bone %s does not exist", p_bone_name));
	ERR_FAIL_COND_V_MSG(!bd->has_scale, Vector3(), vformat("Bone %s doesn't have a scale in this pose", p_bone_name));
	return bd->scale;
}

void EPASPose::set_bone_has_scale(const StringName &p_bone_name, bool p_has_scale) {
	ERR_FAIL_COND_MSG(!has_bone(p_bone_name), vformat("Bone %s does not exist", p_bone_name));
	get_bone_data(p_bone_name)->has_scale = p_has_scale;
}

bool EPASPose::get_bone_has_scale(const StringName &p_bone_name) const {
	ERR_FAIL_COND_V_MSG(!has_bone(p_bone_name), false, vformat("Bone %s does not exist", p_bone_name));
	return get_bone_data(p_bone_name)->has_scale;
}

StringName EPASPose::get_bone_name(const int p_bone_idx) const {
	ERR_FAIL_INDEX_V_MSG(p_bone_idx, bone_datas_v.size(), StringName(), vformat("Bone idx %d is out of range!", p_bone_idx));
	return bone_datas_v[p_bone_idx]->bone_name;
}

Transform3D EPASPose::get_bone_transform(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose) const {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	if (p_base_pose.is_valid()) {
		ERR_FAIL_COND_V_MSG(!p_base_pose->has_bone(p_bone_name), Transform3D(), vformat("Bone %s does not exist in base pose", p_bone_name));
		if (!bd) {
			return p_base_pose->get_bone_transform(p_bone_name);
		}
		return bd->get_transform(p_base_pose->get_bone_data(p_bone_name));
	}
	ERR_FAIL_COND_V_MSG(!bd, Transform3D(), vformat("Bone %s does not exist", p_bone_name));
	ERR_FAIL_COND_V_MSG(!bd->has_position, Transform3D(), vformat("Bone %s doesn't have a rotation in this pose", p_bone_name));
	ERR_FAIL_COND_V_MSG(!bd->has_rotation, Transform3D(), vformat("Bone %s doesn't have a rotation in this pose", p_bone_name));
	ERR_FAIL_COND_V_MSG(!bd->has_scale, Transform3D(), vformat("Bone %s doesn't have a rotation in this pose", p_bone_name));
	return bd->get_transform(nullptr);
}

void EPASPose::flip_along_z() {
	HashSet<String> processed_bones;
	processed_bones.reserve(get_bone_count());
	for (KeyValue<StringName, BoneData *> &kv : bone_datas) {
		String bone_name = kv.key;
		if (processed_bones.has(kv.key)) {
			continue;
		}
		processed_bones.insert(kv.key);
		if (kv.value->has_position) {
			kv.value->position = kv.value->position * Vector3(-1.0, 1.0, 1.0);
		}
		if (kv.value->has_rotation) {
			Basis bas = Basis(kv.value->rotation);
			bas.set_euler(bas.get_euler() * Vector3(1.0, -1.0, -1.0));
			kv.value->rotation = bas.get_rotation_quaternion();
		}

		// L/R bones are flipped and swapped, ordinary bones are just flipped
		StringName other_bone_name;
		if (bone_name.ends_with(".R")) {
			other_bone_name = bone_name.replace(".R", ".L");
		} else if (bone_name.ends_with(".L")) {
			other_bone_name = bone_name.replace(".L", ".R");
		}

		if (!String(other_bone_name).is_empty()) {
			BoneData *other_bone_data = get_bone_data(other_bone_name);
			if (!other_bone_data) {
				create_bone(other_bone_name);
				other_bone_data = get_bone_data(other_bone_name);
			}
			ERR_FAIL_COND(!other_bone_data);
			processed_bones.insert(other_bone_name);
			if (other_bone_data->has_position) {
				other_bone_data->position = other_bone_data->position * Vector3(-1.0, 1.0, 1.0);
			}
			if (other_bone_data->has_rotation) {
				Basis bas = Basis(other_bone_data->rotation);
				bas.set_euler(bas.get_euler() * Vector3(1.0, -1.0, -1.0));
				other_bone_data->rotation = bas.get_rotation_quaternion();
			}
			SWAP(other_bone_data->has_position, kv.value->has_position);
			SWAP(other_bone_data->has_rotation, kv.value->has_rotation);
			SWAP(other_bone_data->position, kv.value->position);
			SWAP(other_bone_data->rotation, kv.value->rotation);
		}
	}
}

void EPASPose::add(const Ref<EPASPose> &p_second_pose, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_output, float p_blend, TypedArray<StringName> p_bone_filter) const {
	for (int i = 0; i < p_base_pose->get_bone_count(); i++) {
		// this is the output bonedata
		StringName bone_name = p_base_pose->get_bone_name(i);

		if (p_bone_filter.size() > 0 && !p_bone_filter.has(bone_name)) {
			continue;
		}

		bool first_has_bone = has_bone(bone_name);
		bool second_has_bone = p_second_pose->has_bone(bone_name);
		Ref<EPASPose> first_pose = Ref<EPASPose>(this);
		Ref<EPASPose> second_pose = p_second_pose;

		if (!first_has_bone && !second_has_bone) {
			// If neither of the poses has the bone we do nothing, the controller will apply the base values here
			continue;
		}

		if (!p_output->has_bone(bone_name)) {
			// Ensure the output pose has this bone
			p_output->create_bone(bone_name);
		}

		if (!first_has_bone) {
			// If a bone is missing we just use the base pose
			first_pose = p_base_pose;
		}
		if (!second_has_bone) {
			// If a bone is missing we just use the base pose
			second_pose = p_base_pose;
		}

		// Interpolate all values with fallback to base pose
		if (first_pose->get_bone_has_position(bone_name) || second_pose->get_bone_has_position(bone_name)) {
			Vector3 final_pos = first_pose->get_bone_position(bone_name, p_base_pose);
			final_pos = final_pos.lerp(final_pos + second_pose->get_bone_position(bone_name, p_base_pose), p_blend);
			p_output->set_bone_position(bone_name, final_pos);
		}

		if (first_pose->get_bone_has_rotation(bone_name) || second_pose->get_bone_has_rotation(bone_name)) {
			Quaternion final_rot = first_pose->get_bone_rotation(bone_name, p_base_pose);
			final_rot = final_rot.slerp(second_pose->get_bone_rotation(bone_name, p_base_pose) * final_rot, p_blend);
			p_output->set_bone_rotation(bone_name, final_rot);
		}

		if (first_pose->get_bone_has_scale(bone_name) || second_pose->get_bone_has_scale(bone_name)) {
			Vector3 final_pos = first_pose->get_bone_scale(bone_name, p_base_pose);
			final_pos = final_pos.lerp(final_pos + second_pose->get_bone_scale(bone_name, p_base_pose), p_blend);
			p_output->set_bone_scale(bone_name, final_pos);
		}
	}
}

void EPASPose::blend(const Ref<EPASPose> &p_second_pose, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_output, float p_blend, TypedArray<StringName> p_bone_filter) const {
	for (int i = 0; i < p_base_pose->get_bone_count(); i++) {
		// this is the output bonedata
		StringName bone_name = p_base_pose->get_bone_name(i);

		if (p_bone_filter.size() > 0 && !p_bone_filter.has(bone_name)) {
			continue;
		}

		bool first_has_bone = has_bone(bone_name);
		bool second_has_bone = p_second_pose->has_bone(bone_name);
		Ref<EPASPose> first_pose = Ref<EPASPose>(this);
		Ref<EPASPose> second_pose = p_second_pose;

		if (!first_has_bone && !second_has_bone) {
			// If neither of the poses has the bone we do nothing, the controller will apply the base values here
			continue;
		}

		if (!p_output->has_bone(bone_name)) {
			// Ensure the output pose has this bone
			p_output->create_bone(bone_name);
		}

		if (!first_has_bone) {
			// If a bone is missing we just use the base pose
			first_pose = p_base_pose;
		}
		if (!second_has_bone) {
			second_pose = p_base_pose;
		}
		// Interpolate all values with fallback to base pose
		if (first_pose->get_bone_has_position(bone_name) || second_pose->get_bone_has_position(bone_name)) {
			Vector3 final_pos = first_pose->get_bone_position(bone_name, p_base_pose);
			final_pos = final_pos.lerp(second_pose->get_bone_position(bone_name, p_base_pose), p_blend);
			p_output->set_bone_position(bone_name, final_pos);
		}

		if (first_pose->get_bone_has_rotation(bone_name) || second_pose->get_bone_has_rotation(bone_name)) {
			Quaternion final_rot = first_pose->get_bone_rotation(bone_name, p_base_pose);
			final_rot = final_rot.slerp(second_pose->get_bone_rotation(bone_name, p_base_pose), p_blend);
			p_output->set_bone_rotation(bone_name, final_rot);
		}

		if (first_pose->get_bone_has_scale(bone_name) || second_pose->get_bone_has_scale(bone_name)) {
			Vector3 final_scale = first_pose->get_bone_scale(bone_name, p_base_pose);
			final_scale = final_scale.lerp(second_pose->get_bone_scale(bone_name, p_base_pose), p_blend);
			p_output->set_bone_scale(bone_name, final_scale);
		}
	}
}

bool EPASPose::has_bone(const StringName &p_bone_name) const {
	return bone_datas.has(p_bone_name);
}

EPASPose::~EPASPose() {
	clear();
}