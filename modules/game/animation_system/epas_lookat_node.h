/**************************************************************************/
/*  epas_lookat_node.h                                                    */
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

#ifndef EPAS_LOOKAT_NODE_H
#define EPAS_LOOKAT_NODE_H

#include "epas_node.h"

class EPASLookatNode : public EPASNode {
	GDCLASS(EPASLookatNode, EPASNode);
	StringName bone_name;
	Vector3 skeleton_forward = Vector3(0.0f, 0.0f, 1.0f);
	Vector3 skeleton_rotation_axis = Vector3(0.0f, 1.0f, 0.0f);
	Vector3 target_world;
	float influence = 1.0f;
	float spring_halflife = 0.15f;
	float max_angle_degrees = 180.0f;

	bool has_target_rotation = false;
	Vector3 quaternion_spring_velocity;
	Quaternion quaternion_spring_value;

protected:
	static void _bind_methods();

public:
	Vector3 get_skeleton_forward() const;
	void set_skeleton_forward(const Vector3 &p_skeleton_forward);
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	EPASLookatNode();

	Vector3 get_skeleton_rotation_axis() const;
	void set_skeleton_rotation_axis(const Vector3 &p_skeleton_rotation_axis);

	Vector3 get_target_world() const;
	void set_target_world(const Vector3 &p_target_world);

	StringName get_bone_name() const;
	void set_bone_name(const StringName &p_bone_name);

	float get_influence() const;
	void set_influence(float p_influence);

	float get_max_angle_degrees() const;
	void set_max_angle_degrees(float p_max_angle_degrees);

	float get_spring_halflife() const;
	void set_spring_halflife(float p_spring_halflife);
	void reset();

#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
};

#endif // EPAS_LOOKAT_NODE_H
