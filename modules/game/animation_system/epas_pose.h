
#ifndef EPAS_POSE_H
#define EPAS_POSE_H

#include "core/math/vector3.h"
#include "core/string/ustring.h"
#include "scene/resources/animation.h"

class EPASPose : public Resource {
	GDCLASS(EPASPose, Resource);

	void clear();

protected:
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;
	static void _bind_methods();

public:
	struct BoneData {
		bool enabled = true;
		String bone_name;
		bool has_position = false;
		Vector3 position;
		bool has_rotation = false;
		Quaternion rotation;
		bool has_scale = false;
		Vector3 scale;
		BoneData(const String &p_bone_name) {
			bone_name = p_bone_name;
		}
		Vector3 get_position(BoneData *p_fallback) {
			if (has_position) {
				return position;
			}
			ERR_FAIL_COND_V(p_fallback == nullptr, Vector3());
			return p_fallback->position;
		}
		Vector3 get_scale(BoneData *p_fallback) {
			if (has_scale) {
				return scale;
			}
			ERR_FAIL_COND_V(p_fallback == nullptr, Vector3(1.0f, 1.0f, 1.0f));
			return p_fallback->scale;
		}
		Quaternion get_rotation(BoneData *p_fallback) {
			if (has_rotation) {
				return rotation;
			}
			ERR_FAIL_COND_V(p_fallback == nullptr, Quaternion());
			return p_fallback->rotation;
		}
	};

private:
	HashMap<String, BoneData *> bone_datas;

public:
	static Ref<EPASPose> from_animation(Ref<Animation> p_animation);
	int get_bone_count() const;
	BoneData *get_bone_data(const String &p_bone_name) const;
	bool has_bone(const String &p_bone_name) const;
	const HashMap<String, BoneData *> get_bone_map();
	BoneData *create_bone(const String &p_bone_name);

	// Special version of above without the pointer return type
	// for GDScript binding
	void create_bone_gd(const String &p_bone_name);
	void set_bone_position(const String &p_bone_name, const Vector3 &p_position);
	void set_bone_rotation(const String &p_bone_name, const Quaternion &p_rotation);
	void set_bone_scale(const String &p_bone_name, const Vector3 &p_scale);

	void reserve(int p_size);

	virtual ~EPASPose();
};

#endif // EPAS_POSE_H