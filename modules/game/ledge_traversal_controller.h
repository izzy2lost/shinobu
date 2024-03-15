/**************************************************************************/
/*  ledge_traversal_controller.h                                          */
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

#ifndef LEDGE_TRAVERSAL_CONTROLLER_H
#define LEDGE_TRAVERSAL_CONTROLLER_H

#include "agent_procedural_anim.h"
#include "debug_geometry.h"
#include "modules/game/agent_parkour.h"
#include "scene/3d/mesh_instance_3d.h"

class HBLedgeTraversalController : public Node3D {
	GDCLASS(HBLedgeTraversalController, Node3D);
	Transform3D limb_transforms[AgentProceduralAnimator::LIMB_MAX];
	Transform3D limb_transforms_predicted[AgentProceduralAnimator::LIMB_MAX];
	bool limb_dangle_status[AgentProceduralAnimator::LIMB_MAX] = { false };
	float radius = 0.25f;
	float height = 1.35f;
	float max_movement_velocity = 0.4f;
	float movement_velocity_target = 0.0f;
	float movement_velocity = 0.0f;
	float movement_acceleration = 0.4f;
	Quaternion out_rotation;
	float curve_offset = 0.0f;
	HBAgentParkourLedge *ledge = nullptr;

	Ref<Curve3D> current_curve;

	Vector3 movement_input;
	Vector3 get_transformed_movement_input() const;
	MeshInstance3D *movement_dir_debug_graphic = nullptr;
	void _update_movement_dir_debug_graphic();
	HBDebugGeometry *debug_geo = nullptr;
	void _handle_limbs();
	Transform3D calculate_limb_trf(HBAgentParkourLedge *p_ledge, float p_agent_radius, float p_limb_offset, float p_agent_offset, const AgentProceduralAnimator::AgentLimb p_limb) const;
	float limb_prediction_multiplier = 0.05f;

protected:
	static void _bind_methods();

public:
	void move_to_offset(float p_offset);
	float get_ledge_offset() const;
	void move_to_ledge(HBAgentParkourLedge *p_ledge, const float p_offset);
	void update(float p_delta);

	Vector3 get_movement_input() const;
	void set_movement_input(const Vector3 &p_movement_input);
	Quaternion get_graphics_rotation() const;
	bool get_limb_is_dangling(AgentProceduralAnimator::AgentLimb p_limb) const;
	Transform3D get_limb_transform(AgentProceduralAnimator::AgentLimb p_limb) const;
	Transform3D get_limb_transform_predicted(AgentProceduralAnimator::AgentLimb p_limb) const;
	float get_movement_velocity() const;
	float get_max_movement_velocity() const { return max_movement_velocity; };
	void set_max_movement_velocity(float p_max_movement_velocity) { max_movement_velocity = p_max_movement_velocity; };
	void calculate_pose(HBAgentParkourLedge *p_ledge, float p_offset, AgentProceduralAnimator::AgentProceduralPose &p_pose) const;

	HBLedgeTraversalController();
};

#endif // LEDGE_TRAVERSAL_CONTROLLER_H
