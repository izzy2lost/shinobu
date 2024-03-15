/**************************************************************************/
/*  game_world.h                                                          */
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

#ifndef GAME_WORLD_H
#define GAME_WORLD_H

#include "modules/game/console_system.h"
#include "modules/game/in_game_ui.h"
#include "scene/3d/node_3d.h"

// The objective of the combat coordinator is to ensure the player is never attacked by two enemies at the same instant
// he can be attacked by two enemies at the same time, which allows him to interrupt his current attack by parrying
class GameCombatCoordinator : public RefCounted {
	// In milliseconds, time since the last attack happened
	//uint64_t last_attack_time = 0;
};
class HBAgent;
class HBPlayerAgent;
class GameWorldState : public RefCounted {
	GDCLASS(GameWorldState, RefCounted);
	static CCommand trigger_alert_cc;

public:
	enum AlertStatus {
		ALERT_CLEAR, // Everything is clear
		ALERT_LOST, // Player has been spotted but isn't in view
		ALERT_SEEN // Player's location is known
	};

protected:
	static void _bind_methods();

private:
	AlertStatus alert_status = GameWorldState::ALERT_CLEAR;
	HBPlayerAgent *player;
	HashSet<ObjectID> agents_in_combat;

	void _on_agent_entered_combat(HBAgent *p_agent);
	void _on_agent_exited_combat(HBAgent *p_agent);
	void agent_entered_tree(HBAgent *p_agent);
	void agent_exited_tree(HBAgent *p_agent);

public:
	AlertStatus get_alert_status() const { return alert_status; }
	void set_alert_status(const AlertStatus p_alert_status) { alert_status = p_alert_status; }
	void set_player(HBPlayerAgent *p_player);
	HashSet<ObjectID> get_agents_in_combat() const { return agents_in_combat; };
	HBPlayerAgent *get_player() const { return player; };
	GameWorldState();
	friend class HBGameWorld;
	friend class HBAgent;
};

VARIANT_ENUM_CAST(GameWorldState::AlertStatus);

// Central game coordinator
class HBGameWorld : public Node3D {
	GDCLASS(HBGameWorld, Node3D);

	Transform3D player_start_transform;
	CanvasLayer *ui_canvas_layer = nullptr;
	HBInGameUI *game_ui = nullptr;
	Ref<GameWorldState> world_state;
	HBPlayerAgent *player = nullptr;

public:
	enum {
		NOTIFICATION_HB_ENTER_GAME_WORLD = 4000,
		NOTIFICATION_HB_EXIT_GAME_WORLD = 4001,
	};

	void _on_node_added(Node *p_node);
	void _on_node_removed(Node *p_node);

	void set_player(HBPlayerAgent *p_player);
	HBPlayerAgent *get_player() const { return player; };

	void set_player_start_transform(const Transform3D &p_transform);
	void spawn_player();
	Ref<GameWorldState> get_game_world_state() const;

protected:
	void _notification(int p_what);
#ifdef DEBUG_ENABLED
	void _debug_notification(int p_what);
#endif
public:
	HBGameWorld();
};

#endif // GAME_WORLD_H
