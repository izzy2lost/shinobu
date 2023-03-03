#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include "agent.h"
#include "state_machine.h"

class HBAgentState : public HBStateMachineState {
	GDCLASS(HBAgentState, HBStateMachineState);

protected:
	HBAgent *get_agent() const;
};

#endif // AGENT_STATE_H
