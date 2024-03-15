/**************************************************************************/
/*  epas_softness_node.h                                                  */
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

#ifndef EPAS_SOFTNESS_NODE_H
#define EPAS_SOFTNESS_NODE_H

#include "../agent.h"
#include "epas_node.h"

class EPASSoftnessNode : public EPASNode {
	GDCLASS(EPASSoftnessNode, EPASNode);
	Vector3 character_prev_velocities[2] = {};
	struct SoftnessMapEntry {
		float softness;
		Vector3 child_spring_velocity;
		Vector3 child_spring_acceleration;
		Vector3 child_prev_positions[2] = {};
		bool has_global_rot = false;
		Quaternion global_rot;
		Vector3 child_spring_position;
	};
	HashMap<StringName, SoftnessMapEntry> softness_map;
	float influence = 0.0f;
	float acceleration_multiplier = 1.0f;
	HBAgent *character = nullptr;

	const static int PLOT_SAMPLES = 90;
	float acceleration_plot[2][90] = { {}, {} };
	float plot_t = 0.0f;
	bool show_plot = false;

protected:
	static void _bind_methods();
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
public:
	void set_bone_softness(const StringName &p_bone_name, float p_softness);
	void set_character(HBAgent *p_agent);
	void set_influence(float p_influence);
	float get_influence() const;
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;

	EPASSoftnessNode();
};

#endif // EPAS_SOFTNESS_NODE_H
