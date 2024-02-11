#ifndef NPC_AGENT_H
#define NPC_AGENT_H

#include "agent.h"
#include "modules/game/safe_godot_obj_id_ref.h"
#include "modules/game/state_machine.h"
#include "agent_state.h"

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
    NODE_CACHE_IMPL(patrol_route, HBRoute);
protected:
    static void _bind_methods();
public:
    NavigationAgent3D *get_navigation_agent() const;
    void set_navigation_target_position(const Vector3 &p_target_position);
    bool move_to_target(float p_delta);
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
