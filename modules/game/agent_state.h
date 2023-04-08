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
	Node3D *get_graphics_node();
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
	bool _handle_parkour_down();
	bool _handle_parkour_up();
	bool _check_wall(PhysicsDirectSpaceState3D::RayResult &p_result);

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

class HBAgentTurnState : public HBAgentState {
	GDCLASS(HBAgentTurnState, HBAgentState);
	Ref<EPASOneshotAnimationNode> animation_node;
	Quaternion starting_rot;
	Quaternion target_rot;
	float anim_length = 0.0f;
	float time = 0.0f;
	void _on_animation_finished();

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void exit() override;
	virtual void process(float p_delta) override;
};

class HBAgentWallrunState : public HBAgentState {
	GDCLASS(HBAgentWallrunState, HBAgentState);
	Ref<EPASOneshotAnimationNode> animation_node;
	EPASOneshotAnimationNodeDebug *dbg = nullptr;

protected:
	virtual void enter(const Dictionary &p_args) override;
	virtual void process(float p_delta) override;
};

class HBAgentWallGrabbedState : public HBAgentState {
	GDCLASS(HBAgentWallGrabbedState, HBAgentState);

	virtual void enter(const Dictionary &p_args) override;
};

#endif // AGENT_STATE_H
