#ifndef HIT_STOP_H
#define HIT_STOP_H

#include "core/object/ref_counted.h"

class HitStopSolver : public RefCounted {
	GDCLASS(HitStopSolver, RefCounted);

	Vector3 direction;

	float duration = 0.0f;
	float current_time = 0.0f;

	float shake_amount = 0.1f;

	Vector3 current_offset;

public:
	void start(Vector3 p_direction, float p_duration);
	void advance(float p_camera_distance, float p_delta);
	Vector3 get_offset() const;
	bool is_done() const;
};

#endif // HIT_STOP_H
