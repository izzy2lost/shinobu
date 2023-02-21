#ifndef EPAS_ANIMATION_NODE_H
#define EPAS_ANIMATION_NODE_H

#include "core/variant/binder_common.h"
#include "modules/game/animation_system/epas_animation.h"
#include "modules/game/animation_system/epas_node.h"

class EPASAnimationNode : public EPASNode {
	GDCLASS(EPASAnimationNode, EPASNode);

public:
	enum PlaybackMode {
		AUTOMATIC = 0,
		MANUAL,
	};

private:
	PlaybackMode playback_mode = PlaybackMode::AUTOMATIC;
	Ref<EPASAnimation> animation;
	EPASAnimation::InterpolationMethod interpolation_method = EPASAnimation::InterpolationMethod::LINEAR;

	float time = 0.0f;

protected:
	static void _bind_methods();

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	void set_animation(Ref<EPASAnimation> p_animation);
	Ref<EPASAnimation> get_animation() const;
	void set_playback_mode(PlaybackMode p_playback_mode);
	PlaybackMode get_playback_mode() const;
	void set_interpolation_method(EPASAnimation::InterpolationMethod p_interpolation_method);
	EPASAnimation::InterpolationMethod get_interpolation_method() const;
	void seek(float p_time);

	EPASAnimationNode();
};

VARIANT_ENUM_CAST(EPASAnimationNode::PlaybackMode);

#endif // EPAS_ANIMATION_NODE_H
