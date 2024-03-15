/**************************************************************************/
/*  epas_node.h                                                           */
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

#ifndef EPAS_NODE_H
#define EPAS_NODE_H

#include "epas_pose.h"
#include "scene/gui/control.h"

class EPASController;

class EPASNode : public RefCounted {
	GDCLASS(EPASNode, RefCounted);

	Vector<Ref<EPASNode>> children;
	StringName node_name;
	EPASController *epas_controller;

private:
protected:
	void set_epas_controller(EPASController *p_epas_controller);
	EPASController *get_epas_controller() const;
	static void _bind_methods();
	Skeleton3D *get_skeleton() const;
	void _set_input_count(int p_count);

public:
	int get_input_count() const;
	virtual void connect_to_input(int p_input, Ref<EPASNode> p_node);
	void process_input_pose(int p_child, const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta);
	virtual void process_node(const Ref<EPASPose> &p_base_pose, Ref<EPASPose> p_target_pose, float p_delta){};
	Ref<EPASNode> get_input(int p_input) const;
#ifdef DEBUG_ENABLED
	virtual void _debug_node_draw() const {};
#endif
	virtual ~EPASNode(){};
	friend class EPASController;
};

#endif // EPAS_NODE_H