/**************************************************************************/
/*  register_types.h                                                      */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez                            */
/*                                                                        */
/* All rights reserved                                                    */
/**************************************************************************/

#ifndef IMGUI_REGISTER_TYPES_H
#define IMGUI_REGISTER_TYPES_H

#include "modules/register_module_types.h"

void initialize_imgui_module(ModuleInitializationLevel p_level);
void uninitialize_imgui_module(ModuleInitializationLevel p_level);
void imgui_module_post_init();

#endif // IMGUI_REGISTER_TYPES_H
