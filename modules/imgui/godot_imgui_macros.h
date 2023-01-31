/**************************************************************************/
/*  godot_imgui_macros.h                                                  */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez                            */
/*                                                                        */
/* All rights reserved                                                    */
/**************************************************************************/

#ifndef GODOT_IMGUI_MACROS_H
#define GODOT_IMGUI_MACROS_H

#ifdef DEBUG_ENABLED

#include "godot_imgui.h"

#define REGISTER_DEBUG(object)                                 \
	GodotImGui *gim = GodotImGui::get_singleton();             \
	if (gim) {                                                 \
		gim->register_debug_object(object->get_instance_id()); \
	}

#define UNREGISTER_DEBUG(object)                                 \
	GodotImGui *gim = GodotImGui::get_singleton();               \
	if (gim) {                                                   \
		gim->unregister_debug_object(object->get_instance_id()); \
	}

#else
#define REGISTER_DEBUG(object)
#define UNREGISTER_DEBUG(object)
#endif

#endif // GODOT_IMGUI_MACROS_H
