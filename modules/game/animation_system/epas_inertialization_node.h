/**************************************************************************/
/*  epas_inertialization_node.h                                           */
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

#ifndef EPAS_INERTIALIZATION_NODE_H
#define EPAS_INERTIALIZATION_NODE_H

#include "../inertialization.h"
#include "epas_node.h"
#include "modules/game/console_system.h"

class EPASInertializationNode : public EPASNode {
	GDCLASS(EPASInertializationNode, EPASNode);
	static CVar inertialization_dump_path_cvar;
	float desired_blend_time = 0.0f;
	Ref<EPASPose> last_frame_pose;
	Ref<EPASPose> last_last_frame_pose;
	bool inertialization_queued = false;
	Ref<EPASPoseInertializer> pose_inertializer;
	TypedArray<StringName> bone_filter;

protected:
	void static _bind_methods();
	void start_inertialization(const Ref<EPASPose> &p_base_pose, const Ref<EPASPose> &p_current_pose, float p_delta);
	void process_input_pose_inertialized(int p_input, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> &p_target_pose, float p_delta);

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	void inertialize(float p_transition_duration = 0.25f, TypedArray<StringName> p_bone_filter = TypedArray<StringName>());
	bool is_inertializing() const;
	EPASInertializationNode();
};

#endif // EPAS_INERTIALIZATION_NODE_H
