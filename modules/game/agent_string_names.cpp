#include "agent_string_names.h"

AgentStringNames *AgentStringNames::singleton = nullptr;

AgentStringNames::AgentStringNames() :
		move_state(StaticCString::create("Move")),
		vault_state(StaticCString::create("Vault")),
		turn_state(StaticCString::create("Turn")),
		wallrun_state(StaticCString::create("Wallrun")),
		ledge_grabbed_state(StaticCString::create("LedgeGrabbed")),
		fall_state(StaticCString::create("Fall")),
		ledge_get_up_state(StaticCString::create("LedgeGetUp")),
		wall_parkour_state(StaticCString::create("WallParkour")),
		autojump_state(StaticCString::create("Autojump")),
		beam_walk_state(StaticCString::create("BeamWalk")),
		root_motion_state(StaticCString::create("RootMotion")),
		right_foot_ik_node(StaticCString::create("RightFootIK")),
		left_foot_ik_node(StaticCString::create("LeftFootIK")),
		ledge_to_ground_node(StaticCString::create("LedgeToGround")) {
	singleton = this;
}
