/**************************************************************************/
/*  in_game_ui.cpp                                                        */
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

#include "in_game_ui.h"
#include "modules/game/player_agent.h"

void HBInGameUI::_bind_methods() {
	GDVIRTUAL_BIND(_player_health_changed, "old_health", "new_health", "max_health");
}

void HBInGameUI::_on_player_damage_received(int p_old_health, int p_new_health) {
	GDVIRTUAL_CALL(_player_health_changed, p_old_health, p_new_health, player_agent->get_max_health());
}

void HBInGameUI::set_player_agent(HBPlayerAgent *p_player_agent) {
	DEV_ASSERT(p_player_agent != nullptr);
	player_agent = p_player_agent;
	player_agent->connect("damage_received", callable_mp(this, &HBInGameUI::_on_player_damage_received));
	GDVIRTUAL_CALL(_player_health_changed, p_player_agent->get_health(), p_player_agent->get_health(), p_player_agent->get_max_health());
}
