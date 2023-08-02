#ifndef GAME_MAIN_LOOP_H
#define GAME_MAIN_LOOP_H

#include "scene/main/scene_tree.h"

class HBGameMainLoop : public SceneTree {
	_THREAD_SAFE_CLASS_
	GDCLASS(HBGameMainLoop, SceneTree);

	virtual void initialize() override;
	void change_scene(Node *p_new_scene);
	virtual bool process(double p_time) override;
	virtual bool physics_process(double p_time) override;

	HBGameMainLoop();

public:
	void enable_fp_exceptions();
};

#endif