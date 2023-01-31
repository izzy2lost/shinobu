
#ifndef PLAYER_ACTOR_H
#define PLAYER_ACTOR_H

#include "actor.h"
#include "core/math/vector2.h"

class HBPlayerActor : public HBActor {
	GDCLASS(HBPlayerActor, HBActor);

protected:
	virtual Vector3 get_input() const override;
	HBPlayerActor();
};

#endif // PLAYER_ACTOR_H
