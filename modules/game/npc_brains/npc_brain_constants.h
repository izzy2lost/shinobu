#ifndef BRAIN_ATOMS_H
#define BRAIN_ATOMS_H

#include "../agent_constants.h"
#include "npc_approach_player_action.h"

namespace NPCBrainConstants {
//--- GOAP action names
static constexpr const char *APPROACH_PLAYER_ATTACK_RANGE_ACTION_NAME = "approach_player_attack_range";
static constexpr const char *APPROACH_PLAYER_COMBAT_RANGE_ACTION_NAME = "approach_player_combat_range";
static constexpr const char *ATTACK_PLAYER_ACTION_NAME = "attack_player";

//--- Atoms used for GOAP system state
// This means we are very close to the player
static constexpr const char *IN_ATTACK_RANGE_TO_PLAYER = "in_attack_range_to_player";
// This means that we are in the vicinity of the player, not necessarily right next to him
static constexpr const char *IN_COMBAT_RANGE_TO_PLAYER = "in_combat_range_to_player";
static constexpr float COMBAT_RANGE_TARGET_DISTANCE = HBAgentConstants::AGENT_RADIUS * 10.0f;
static constexpr float ATTACK_RANGE_TARGET_DISTANCE = HBAgentConstants::AGENT_RADIUS * 0.5f;
static constexpr const char *PLAYER_DEAD_ATOM = "player_dead";

// Approach state initialization settings
static constexpr GOAPApproachPlayerAction::ApproachPlayerActionSettings in_attack_range_approach_settings = {
	.target_distance = ATTACK_RANGE_TARGET_DISTANCE,
	.player_max_target_deviation = HBAgentConstants::AGENT_RADIUS * 1.0f,
	.action_name = (char *)APPROACH_PLAYER_ATTACK_RANGE_ACTION_NAME,
	.atom_name = (char *)IN_ATTACK_RANGE_TO_PLAYER,
	.should_target_player = true
};
static constexpr GOAPApproachPlayerAction::ApproachPlayerActionSettings in_combat_range_approach_settings = {
	.target_distance = COMBAT_RANGE_TARGET_DISTANCE,
	.player_max_target_deviation = HBAgentConstants::AGENT_RADIUS * 1.0f,
	.action_name = (char *)APPROACH_PLAYER_COMBAT_RANGE_ACTION_NAME,
	.atom_name = (char *)IN_COMBAT_RANGE_TO_PLAYER,
	.should_target_player = false
};
} //namespace NPCBrainConstants

#endif // BRAIN_ATOMS_H
