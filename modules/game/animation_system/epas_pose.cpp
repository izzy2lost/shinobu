#include "epas_pose.h"

void EPASPose::_bind_methods() {
	ClassDB::bind_static_method("EPASPose", D_METHOD("from_animation", "animation"), &EPASPose::from_animation);
	ClassDB::bind_method(D_METHOD("set_bone_position", "bone_name", "position"), &EPASPose::set_bone_position);
	ClassDB::bind_method(D_METHOD("set_bone_rotation", "bone_name", "rotation"), &EPASPose::set_bone_rotation);
	ClassDB::bind_method(D_METHOD("set_bone_scale", "bone_name", "scale"), &EPASPose::set_bone_scale);
	ClassDB::bind_method(D_METHOD("create_bone", "bone_name"), &EPASPose::create_bone_gd);
}

Ref<EPASPose> EPASPose::from_animation(Ref<Animation> p_animation) {
	HashMap<String, BoneData *> bone_poses;
	for (int ti = 0; ti < p_animation->get_track_count(); ti++) {
		NodePath track_path = p_animation->track_get_path(ti);

		if (track_path.get_subname_count() == 0) {
			continue;
		}

		// TODO: is it fine that we assume that the first subname is the
		// bone name?

		String bone_name = p_animation->track_get_path(ti).get_subname(0);

		if (!bone_poses.has(bone_name)) {
			bone_poses[bone_name] = memnew(BoneData(bone_name));
			bone_poses[bone_name]->bone_name = bone_name;
		}

		switch (p_animation->track_get_type(ti)) {
			case Animation::TrackType::TYPE_POSITION_3D: {
				bone_poses[bone_name]->has_position = true;
				p_animation->position_track_get_key(ti, 0, &(bone_poses[bone_name]->position));
			} break;
			case Animation::TrackType::TYPE_ROTATION_3D: {
				bone_poses[bone_name]->has_rotation = true;
				p_animation->rotation_track_get_key(ti, 0, &(bone_poses[bone_name]->rotation));
			} break;
			case Animation::TrackType::TYPE_SCALE_3D: {
				bone_poses[bone_name]->has_scale = true;
				p_animation->scale_track_get_key(ti, 0, &(bone_poses[bone_name]->scale));
			} break;
			default: {
			}; break;
		}
	}

	Ref<EPASPose> out_pose = memnew(EPASPose);
	for (const KeyValue<String, BoneData *> &pose : bone_poses) {
		if (pose.value->has_position || pose.value->has_rotation || pose.value->has_scale) {
			out_pose->bone_datas.insert(pose.key, pose.value);
		}
	}
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
				bone_data->has_position = data.has("position");
				bone_data->has_rotation = data.has("rotation");
				bone_data->has_scale = data.has("scale");
				bone_data->bone_name = key;
				if (bone_data->has_position) {
					bone_data->position = data["position"];
				}
				if (bone_data->has_scale) {
					bone_data->scale = data["scale"];
				}
				if (bone_data->has_rotation) {
					bone_data->position = data["scale"];
				}
				bone_datas.insert(bone_data->bone_name, bone_data);
			}
		}
		return true;
	}

	return false;
}

bool EPASPose::_get(const StringName &p_name, Variant &r_ret) const {
	if (p_name == SNAME("pose_data")) {
		Dictionary dic_out;
		for (const KeyValue<String, BoneData *> &kv : bone_datas) {
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
	for (KeyValue<String, BoneData *> kv : bone_datas) {
		memdelete(kv.value);
	}
	bone_datas.clear();
}

int EPASPose::get_bone_count() const {
	return bone_datas.size();
}

EPASPose::BoneData *EPASPose::get_bone_data(const String &p_bone_name) const {
	EPASPose::BoneData *const *pp = bone_datas.getptr(p_bone_name);
	return pp != nullptr ? *pp : nullptr;
}

const HashMap<String, EPASPose::BoneData *> EPASPose::get_bone_map() {
	return bone_datas;
}

void EPASPose::reserve(int p_size) {
	bone_datas.reserve(p_size);
}

EPASPose::BoneData *EPASPose::create_bone(const String &p_bone_name) {
	ERR_FAIL_COND_V_MSG(bone_datas.has(p_bone_name), nullptr, vformat("Bone %s already exists", p_bone_name));
	return bone_datas.insert(p_bone_name, memnew(BoneData(p_bone_name)))->value;
}

void EPASPose::create_bone_gd(const String &p_bone_name) {
	create_bone(p_bone_name);
}

void EPASPose::set_bone_position(const String &p_bone_name, const Vector3 &p_position) {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	ERR_FAIL_COND_MSG(!bd, vformat("Bone %s does not exist", p_bone_name));
	bd->has_position = true;
	bd->position = p_position;
}

void EPASPose::set_bone_rotation(const String &p_bone_name, const Quaternion &p_rotation) {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	ERR_FAIL_COND_MSG(!bd, vformat("Bone %s does not exist", p_bone_name));
	bd->has_rotation = true;
	bd->rotation = p_rotation;
}

void EPASPose::set_bone_scale(const String &p_bone_name, const Vector3 &p_scale) {
	EPASPose::BoneData *bd = get_bone_data(p_bone_name);
	ERR_FAIL_COND_MSG(!bd, vformat("Bone %s does not exist", p_bone_name));
	bd->has_scale = true;
	bd->scale = p_scale;
}

EPASPose::~EPASPose() {
	clear();
}