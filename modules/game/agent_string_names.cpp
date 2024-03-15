/**************************************************************************/
/*  agent_string_names.cpp                                                */
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

#include "agent_string_names.h"

AgentStringNames *AgentStringNames::singleton = nullptr;

void AgentStringNames::_bind_methods() {
}

AgentStringNames::AgentStringNames() :
		move_state(StaticCString::create("Move")),
		turn_state(StaticCString::create("Turn")),
		ledge_grabbed_state(StaticCString::create("LedgeGrabbed")),
		fall_state(StaticCString::create("Fall")),
		ledge_get_up_state(StaticCString::create("LedgeGetUp")),
		wall_parkour_state(StaticCString::create("WallParkour")),
		beam_walk_state(StaticCString::create("BeamWalk")),
		root_motion_state(StaticCString::create("RootMotion")),
		wall_parkour_transition_state(StaticCString::create("WallParkourTransitionState")),
		combat_move_state(StaticCString::create("CombatMove")),
		combat_attack_state(StaticCString::create("CombatAttack")),
		combat_hit_state(StaticCString::create("CombatHit")),
		dead_state(StaticCString::create("Dead")),

		right_hand_ik_node(StaticCString::create("RightHandIK")),
		left_hand_ik_node(StaticCString::create("LeftHandIK")),
		right_foot_ik_node(StaticCString::create("RightFootIK")),
		left_foot_ik_node(StaticCString::create("LeftFootIK")),

		ledge_getup_animation_node(StaticCString::create("LedgeGetUp")),
		ledge_to_ground_animation_node(StaticCString::create("LedgeToGround")),
		ledge_hop_up_animation_node(StaticCString::create("UNUSED")),
		standing_drop_to_ledge_animation_node(StaticCString::create("SurfaceToLedge")),
		standing_jump_to_ledge_animation_node(StaticCString::create("StandingToLedge")),
		short_hop_animation_node(StaticCString::create("ShortHop")),
		back_eject_to_ledge_animation_node(StaticCString::create("BackEjectToLedge")),
		back_eject_to_surface_animation_node(StaticCString::create("BackEjectToSurface")),
		wallparkour_up_long_jump_animation_node(StaticCString::create("WallParkourUpLongJump")),
		wallparkour_down_long_jump_animation_node(StaticCString::create("WallParkourDownLongJump")),
		wallparkour_left_long_jump_animation_node(StaticCString::create("WallParkourLeftLongJump")),
		wallparkour_right_long_jump_animation_node(StaticCString::create("WallParkourRightLongJump")),
		sword_parry_animation_node(StaticCString::create("SwordParry")),
		roll_animation_node(StaticCString::create("Roll")),

		wall_parkour_cat_animation_node(StaticCString::create("WallParkourCat")),
		wall_parkour_cat_point_wp_name(StaticCString::create("ParkourPoint")),

		lookat_head_node_name(StaticCString::create("HeadLookAt")),
		lookat_torso_node_name(StaticCString::create("TorsoLookAt")) {
	singleton = this;
}
