/**************************************************************************/
/*  epas_animation_node.h                                                 */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez (EIRTeam).                 */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

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
		GLOBAL_TIME
	};

private:
	PlaybackMode playback_mode = PlaybackMode::AUTOMATIC;
	Ref<EPASAnimation> animation;
	EPASAnimation::InterpolationMethod interpolation_method = EPASAnimation::InterpolationMethod::LINEAR;

	float time = 0.0f;
	bool looping_enabled = false;

protected:
	static void _bind_methods();

public:
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const override;
#endif
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta) override;
	virtual void interpolate(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_time);
	void set_animation(Ref<EPASAnimation> p_animation);
	Ref<EPASAnimation> get_animation() const;
	void set_playback_mode(PlaybackMode p_playback_mode);
	PlaybackMode get_playback_mode() const;
	void set_interpolation_method(EPASAnimation::InterpolationMethod p_interpolation_method);
	EPASAnimation::InterpolationMethod get_interpolation_method() const;
	void seek(float p_time);
	float get_time();

	bool get_looping_enabled() const;
	void set_looping_enabled(bool p_looping_enabled);
};

VARIANT_ENUM_CAST(EPASAnimationNode::PlaybackMode);

#endif // EPAS_ANIMATION_NODE_H
