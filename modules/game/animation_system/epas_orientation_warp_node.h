/**************************************************************************/
/*  epas_orientation_warp_node.h                                          */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
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

#ifndef EPAS_ORIENTATION_WARP_NODE_H
#define EPAS_ORIENTATION_WARP_NODE_H

#include "modules/game/animation_system/epas_node.h"
class EPASOrientationWarpNode : public EPASNode {
	GDCLASS(EPASOrientationWarpNode, EPASNode);
	StringName foot_ik_node_left;
	StringName foot_ik_node_right;
	StringName pelvis_bone_name;

	Vector<StringName> spine_bones;

	float orientation_spring_velocity = 0.0f;
	float _orientation_angle = 0.0f;
	float orientation_angle = 0.0f;

protected:
	static void _bind_methods();
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	StringName get_foot_ik_node_left() const;
	void set_foot_ik_node_left(const StringName &p_foot_ik_node_left);

	void set_spine_bones(TypedArray<StringName> p_spine_bones) {
		spine_bones.clear();

		for (int i = 0; i < p_spine_bones.size(); i++) {
			spine_bones.push_back(p_spine_bones[i]);
		}
	};
	TypedArray<StringName> get_spine_bones() const {
		TypedArray<StringName> out;
		for (int i = 0; i < spine_bones.size(); i++) {
			out.push_back(spine_bones[i]);
		}

		return out;
	};

	StringName get_foot_ik_node_right() const;
	void set_foot_ik_node_right(const StringName &p_foot_ik_node_right);

	float get_orientation_angle() const;
	void set_orientation_angle(float p_orientation_angle);

	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	StringName get_pelvis_bone_name() const { return pelvis_bone_name; }
	void set_pelvis_bone_name(const StringName &pelvis_bone_name_) { pelvis_bone_name = pelvis_bone_name_; }

	EPASOrientationWarpNode();
};

#endif // EPAS_ORIENTATION_WARP_NODE_H
