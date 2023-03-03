
#ifndef PLAYER_AGENT_H
#define PLAYER_AGENT_H

#include "agent.h"
#include "core/math/vector2.h"

class HBPlayerAgent : public HBAgent {
	GDCLASS(HBPlayerAgent, HBAgent);

protected:
	virtual Vector3 get_input() const override;
	HBPlayerAgent();
};

#endif // PLAYER_AGENT_H
