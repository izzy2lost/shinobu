
#ifndef PLAYER_AGENT_H
#define PLAYER_AGENT_H

#include "agent.h"
#include "modules/game/swansong_gltf_extension.h"
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
	HBPlayerAgent();
};

class HBInfoPlayerStart : public Node3D, public TBLoaderEntity {
	GDCLASS(HBInfoPlayerStart, Node3D);

public:
	static StringName get_entity_name() {
		return "info_player_start";
	}
	virtual void _editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) override;
};

#endif // PLAYER_AGENT_H
