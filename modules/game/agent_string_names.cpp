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

		wall_parkour_cat_animation_node(StaticCString::create("WallParkourCat")),
		wall_parkour_cat_point_wp_name(StaticCString::create("ParkourPoint")),
		
		lookat_head_node_name(StaticCString::create("HeadLookAt")),
		lookat_torso_node_name(StaticCString::create("TorsoLookAt")) {
	singleton = this;
}
