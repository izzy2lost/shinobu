#ifndef NPC_AGENT_H
#define NPC_AGENT_H

#include "agent.h"
#include "agent_state.h"
#include "modules/game/console_system.h"
#include "modules/game/state_machine.h"

class HBRoute : public Node3D, public TBLoaderEntity {
	GDCLASS(HBRoute, Node3D);

	Ref<Curve3D> route;

protected:
	static void _bind_methods();

public:
	virtual void _editor_build(const EntityCompileInfo &p_info, const HashMap<StringName, EntityCompileInfo> &p_entities) override;
	static StringName get_entity_name() {
		return "info_route";
	}

	Ref<Curve3D> get_route() const { return route; }
	void set_route(const Ref<Curve3D> &p_route) { route = p_route; }
};

class HBNPCAgent : public HBAgent {
	GDCLASS(HBNPCAgent, HBAgent);
	NavigationAgent3D *navigation_agent = nullptr;
	static CVar show_pathfinding;
	NODE_CACHE_IMPL(patrol_route, HBRoute);

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	void _on_show_pathfinding_updated();
	NavigationAgent3D *get_navigation_agent() const;
	void set_navigation_target_position(const Vector3 &p_target_position);
	bool move_to_target(float p_delta, bool p_run = false);
	HBNPCAgent();
};

class HBNPCAgentController : public HBStateMachine {
	GDCLASS(HBNPCAgentController, HBStateMachine);
};

class HBNPCAgentPatrol : public HBAgentState {
	GDCLASS(HBNPCAgentPatrol, HBAgentState);

public:
	virtual void physics_process(float p_delta) override;
};

#endif // NPC_AGENT_H
