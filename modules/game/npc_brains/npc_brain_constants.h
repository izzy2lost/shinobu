/**************************************************************************/
/*  npc_brain_constants.h                                                 */
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
