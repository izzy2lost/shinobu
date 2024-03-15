/**************************************************************************/
/*  epas_pose.h                                                           */
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

#ifndef EPAS_POSE_H
#define EPAS_POSE_H

#include "core/math/quaternion.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/string/ustring.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/resources/animation.h"

class EPASPose : public Resource {
	GDCLASS(EPASPose, Resource);
	RES_BASE_EXTENSION("epos");

protected:
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;
	static void _bind_methods();

public:
	struct BoneData {
		bool enabled = true;
		StringName bone_name;
		bool has_position = false;
		Vector3 position;
		bool has_rotation = false;
		Quaternion rotation;
		bool has_scale = false;
		Vector3 scale;
		BoneData(const StringName &p_bone_name) {
			bone_name = p_bone_name;
		}
		Vector3 get_position(const BoneData *p_fallback) const {
			if (has_position) {
				return position;
			}
			ERR_FAIL_COND_V(p_fallback == nullptr, Vector3());
			return p_fallback->position;
		}
		Vector3 get_scale(const BoneData *p_fallback) const {
			if (has_scale) {
				return scale;
			}
			ERR_FAIL_COND_V(p_fallback == nullptr, Vector3(1.0f, 1.0f, 1.0f));
			return p_fallback->scale;
		}
		Quaternion get_rotation(const BoneData *p_fallback) const {
			if (has_rotation) {
				return rotation;
			}
			ERR_FAIL_COND_V(p_fallback == nullptr, Quaternion());
			return p_fallback->rotation;
		}
		Transform3D get_transform(const BoneData *p_fallback) const {
			Transform3D trf;
			trf.origin = get_position(p_fallback);
			trf.basis.set_quaternion_scale(get_rotation(p_fallback), get_scale(p_fallback));
			return trf;
		}
		void interpolate_with(const BoneData *p_second, BoneData *p_output, const BoneData *p_base, float p_blend) const {
			if (p_second->has_position) {
				Vector3 first_pos = get_position(p_base);
				p_output->has_position = true;
				p_output->position = first_pos.lerp(p_second->position, p_blend);
			}
			if (p_second->has_rotation) {
				Quaternion first_rotation = get_rotation(p_base);
				p_output->has_rotation = true;
				p_output->rotation = first_rotation.slerp(p_second->rotation, p_blend);
			}
			if (p_second->has_scale) {
				Vector3 first_scale = get_scale(p_base);
				p_output->has_scale = true;
				p_output->scale = first_scale.lerp(p_second->scale, p_blend);
			}
		}
	};

private:
	HashMap<StringName, BoneData *> bone_datas;
	// This vector is used so we can access bone datas by index
	Vector<BoneData *> bone_datas_v;
	BoneData *get_bone_data(const StringName &p_bone_name) const;
	const HashMap<StringName, BoneData *> get_bone_map() const;

public:
	int get_bone_count() const;
	bool has_bone(const StringName &p_bone_name) const;
	void create_bone(const StringName &p_bone_name);

	// Special version of above without the pointer return type
	// for GDScript binding
	void create_bone_gd(const StringName &p_bone_name);

	void flip_along_z();

	void set_bone_position(const StringName &p_bone_name, const Vector3 &p_position);
	Vector3 get_bone_position(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pos = Ref<EPASPose>()) const;
	void set_bone_has_position(const StringName &p_bone_name, bool p_has_position);
	bool get_bone_has_position(const StringName &p_bone_name) const;

	void set_bone_rotation(const StringName &p_bone_name, const Quaternion &p_rotation);
	Quaternion get_bone_rotation(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose = Ref<EPASPose>()) const;
	void set_bone_has_rotation(const StringName &p_bone_name, bool p_has_rotation);
	bool get_bone_has_rotation(const StringName &p_bone_name) const;

	void set_bone_scale(const StringName &p_bone_name, const Vector3 &p_scale);
	Vector3 get_bone_scale(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose = Ref<EPASPose>()) const;
	void set_bone_has_scale(const StringName &p_bone_name, bool p_has_scale);
	bool get_bone_has_scale(const StringName &p_bone_name) const;

	StringName get_bone_name(const int p_bone_idx) const;

	Transform3D get_bone_transform(const StringName &p_bone_name, const Ref<EPASPose> &p_base_pose = Ref<EPASPose>()) const;
	void reserve(int p_size);
	Transform3D calculate_bone_global_transform(const StringName &p_bone_name, const Skeleton3D *p_skel, const Ref<EPASPose> p_base_pose = Ref<EPASPose>()) const;

	void add(const Ref<EPASPose> &p_second_pose, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_output, float p_blend, TypedArray<StringName> p_bone_filter = TypedArray<StringName>()) const;
	void blend(const Ref<EPASPose> &p_second_pose, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_output, float p_blend, TypedArray<StringName> p_bone_filter = TypedArray<StringName>()) const;

	void clear();

	virtual ~EPASPose();
};

#endif // EPAS_POSE_H