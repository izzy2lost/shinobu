#ifndef GAME_MAIN_LOOP_H
#define GAME_MAIN_LOOP_H

#include "scene/main/scene_tree.h"

class HBGameMainLoop : public SceneTree {
	GDCLASS(HBGameMainLoop, SceneTree);

	virtual void initialize() override;
	void change_scene(Node *p_new_scene);
	HBGameMainLoop();
};

#endif