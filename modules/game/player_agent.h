
#ifndef PLAYER_AGENT_H
#define PLAYER_AGENT_H

#include "agent.h"

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

#endif // PLAYER_AGENT_H
