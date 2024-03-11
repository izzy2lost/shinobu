/**************************************************************************/
/*  game_main_loop.h                                                      */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
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
	static HBGameMainLoop *singleton;

	virtual void initialize() override;
	void change_scene(Node *p_new_scene);
	virtual bool process(double p_time) override;
	virtual bool physics_process(double p_time) override;
	virtual void finalize() override;

public:
	HBGameWorld *get_game_world() const;
	void enable_fp_exceptions();
	static HBGameMainLoop *get_singleton();
	HBGameMainLoop();
};

#endif