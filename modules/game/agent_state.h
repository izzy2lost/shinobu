#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include "agent.h"
#include "animation_system/epas_oneshot_animation_node.h"
#include "animation_system/epas_transition_node.h"
#include "debug_geometry.h"
#include "state_machine.h"
class HBAgentState : public HBStateMachineState {
	GDCLASS(HBAgentState, HBStateMachineState);

public:
	HBAgent *get_agent() const;
	EPASController *get_epas_controller() const;
	Ref<EPASTransitionNode> get_movement_transition_node() const;
	Skeleton3D *get_skeleton() const;
};

class HBAgentMoveState : public HBAgentState {
	GDCLASS(HBAgentMoveState, HBAgentState);
#ifdef DEBUG_ENABLED
	HBDebugGeometry *debug_geo;
	bool show_debug_geometry = false;
#endif
protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void physics_process(float p_delta) override;
	virtual void debug_draw() const override;

public:
	HBAgentMoveState();
};

class HBAgentVaultState : public HBAgentState {
	GDCLASS(HBAgentVaultState, HBAgentState);

	Ref<EPASOneshotAnimationNode> animation_node;
	Vector3 prev_position;

private:
	void _on_animation_finished();

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void process(float p_delta) override;
};

#endif // AGENT_STATE_H
