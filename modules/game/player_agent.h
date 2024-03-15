/**************************************************************************/
/*  player_agent.h                                                        */
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

#ifndef PLAYER_AGENT_H
#define PLAYER_AGENT_H

#include "agent.h"
#include "modules/tbloader/src/tb_loader_singleton.h"

class HBPlayerAgentController : public Node {
	GDCLASS(HBPlayerAgentController, Node);
	ObjectID agent_node_cache;
	NodePath agent_node;

	HBAgent *_get_agent() const;
	void _update_agent_node_cache();

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	NodePath get_agent_node() const;
	void set_agent_node(const NodePath &p_agent_node);
	HBPlayerAgentController();
	~HBPlayerAgentController();
};

class HBPlayerAgent : public HBAgent {
	GDCLASS(HBPlayerAgent, HBAgent);

protected:
	void _notification(int p_what);
	HBPlayerAgent();
};

class HBInfoPlayerStart : public Node3D, public TBLoaderEntity {
	GDCLASS(HBInfoPlayerStart, Node3D);

protected:
	void _notification(int p_what);

public:
	static StringName get_entity_name() {
		return "info_player_start";
	}
	virtual void _editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) override;
};

#endif // PLAYER_AGENT_H
