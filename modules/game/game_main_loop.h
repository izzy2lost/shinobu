#ifndef GAME_MAIN_LOOP_H
#define GAME_MAIN_LOOP_H

#include "modules/game/console_system.h"
#include "scene/main/scene_tree.h"

class HBGameWorld;

class HBGameMainLoop : public SceneTree {
	_THREAD_SAFE_CLASS_
	GDCLASS(HBGameMainLoop, SceneTree);

	static CCommand quit_command;
	HBGameWorld *game_world = nullptr;

	virtual void initialize() override;
	void change_scene(Node *p_new_scene);
	virtual bool process(double p_time) override;
	virtual bool physics_process(double p_time) override;
	virtual void finalize() override;

	HBGameMainLoop();

public:
	HBGameWorld *get_game_world() const;
	void enable_fp_exceptions();
};

#endif