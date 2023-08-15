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
		ledge_drop_state(StaticCString::create("LedgeDrop")) {
	singleton = this;
}
