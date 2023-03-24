/**************************************************************************/
/*  godot_imgui.h                                                         */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez                            */
/*                                                                        */
/* All rights reserved                                                    */
/**************************************************************************/

#ifndef GODOT_IMGUI_H
#define GODOT_IMGUI_H
#ifdef DEBUG_ENABLED
#include "ImGuizmo.h"
#include "imgui.h"
#include "imgui_neo_sequencer.h"
#include "scene/gui/subviewport_container.h"
#include "scene/main/viewport.h"
#include "thirdparty/fonts/imgui_fonts.h"
#include <cstdint>

class GodotImGui : public SubViewportContainer {
	GDCLASS(GodotImGui, SubViewportContainer);

private:
	static GodotImGui *singleton;
	SubViewport *viewport = nullptr;
	struct ObjectDebugInfo {
		bool debug_enabled = false;
	};
	HashMap<ObjectID, ObjectDebugInfo> debug_status;
	bool debug_active = false;
	bool show_demo_window = false;
	bool show_overlay = true;

	RID framebuffer;
	RID shader;
	RID pipeline;
	RID sampler;
	RID index_buffer;
	int index_buffer_size = 0;
	RID vertex_buffer;
	int vertex_buffer_size = 0;
	HashSet<uint64_t> used_textures;
	HashMap<uint64_t, RID> uniform_sets;
	Ref<ImageTexture> font_texture;
	PackedFloat32Array push_constant_buffer;
	RenderingDevice::VertexFormatID vertex_format;

	void _init_imgui();
	void _begin_frame();
	void _end_frame();
	void _recreate_framebuffer();
	void _render_draw_data(ImDrawData *p_draw_data);
	void _setup_buffers(ImDrawData *p_draw_data);
	void _show_overlay();
	ImGuiKey _map_to_imgui_key(const Key &p_key);

protected:
	void _notification(int p_what);
	virtual void gui_input(const Ref<InputEvent> &p_event) override;
	virtual void unhandled_key_input(const Ref<InputEvent> &p_event) override;
	virtual void unhandled_input(const Ref<InputEvent> &p_event) override;

public:
	static GodotImGui *get_singleton() { return singleton; };

	// Use this to register debug objects
	void register_debug_object(const ObjectID &p_object_id);
	void unregister_debug_object(const ObjectID &p_object_id);
	// Use this to check if debugging is enabled for this object before drawing using imgui
	bool is_debugging_enabled(const ObjectID &p_object_id) const;
	bool begin_debug_window(const Object *p_object);
	bool is_debug_enabled(const Object *p_object) const;

	void set_enable_overlay(bool p_enable);

	GodotImGui();
	~GodotImGui();
};
#endif // DEBUG_ENABLED
#endif // GODOT_IMGUI_H
