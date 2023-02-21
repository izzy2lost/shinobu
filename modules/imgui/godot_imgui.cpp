/**************************************************************************/
/*  godot_imgui.cpp                                                       */
/**************************************************************************/
/*                         This file is part of:                          */
/*                               SWANSONG                                 */
/*                          https://eirteam.moe                           */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román Núñez                            */
/*                                                                        */
/* All rights reserved                                                    */
/**************************************************************************/

#include "godot_imgui.h"

#include "imnodes.h"
#include "main/performance.h"
#include "scene/gui/label.h"
#include "servers/rendering/rendering_device_binds.h"

GodotImGui *GodotImGui::singleton = nullptr;

static void embrace_the_darkness() {
	ImVec4 *colors = ImGui::GetStyle().Colors;
	colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
	colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
	colors[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_PopupBg] = ImVec4(0.19f, 0.19f, 0.19f, 0.92f);
	colors[ImGuiCol_Border] = ImVec4(0.19f, 0.19f, 0.19f, 0.29f);
	colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.24f);
	colors[ImGuiCol_FrameBg] = ImVec4(0.05f, 0.05f, 0.05f, 0.54f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.19f, 0.19f, 0.19f, 0.54f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
	colors[ImGuiCol_TitleBg] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_TitleBgActive] = ImVec4(0.06f, 0.06f, 0.06f, 1.00f);
	colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_MenuBarBg] = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
	colors[ImGuiCol_ScrollbarBg] = ImVec4(0.05f, 0.05f, 0.05f, 0.54f);
	colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.34f, 0.34f, 0.34f, 0.54f);
	colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.40f, 0.40f, 0.40f, 0.54f);
	colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.56f, 0.56f, 0.56f, 0.54f);
	colors[ImGuiCol_CheckMark] = ImVec4(0.33f, 0.67f, 0.86f, 1.00f);
	colors[ImGuiCol_SliderGrab] = ImVec4(0.34f, 0.34f, 0.34f, 0.54f);
	colors[ImGuiCol_SliderGrabActive] = ImVec4(0.56f, 0.56f, 0.56f, 0.54f);
	colors[ImGuiCol_Button] = ImVec4(0.05f, 0.05f, 0.05f, 0.54f);
	colors[ImGuiCol_ButtonHovered] = ImVec4(0.19f, 0.19f, 0.19f, 0.54f);
	colors[ImGuiCol_ButtonActive] = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
	colors[ImGuiCol_Header] = ImVec4(0.00f, 0.00f, 0.00f, 0.52f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.00f, 0.00f, 0.00f, 0.36f);
	colors[ImGuiCol_HeaderActive] = ImVec4(0.20f, 0.22f, 0.23f, 0.33f);
	colors[ImGuiCol_Separator] = ImVec4(0.28f, 0.28f, 0.28f, 0.29f);
	colors[ImGuiCol_SeparatorHovered] = ImVec4(0.44f, 0.44f, 0.44f, 0.29f);
	colors[ImGuiCol_SeparatorActive] = ImVec4(0.40f, 0.44f, 0.47f, 1.00f);
	colors[ImGuiCol_ResizeGrip] = ImVec4(0.28f, 0.28f, 0.28f, 0.29f);
	colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.44f, 0.44f, 0.44f, 0.29f);
	colors[ImGuiCol_ResizeGripActive] = ImVec4(0.40f, 0.44f, 0.47f, 1.00f);
	colors[ImGuiCol_Tab] = ImVec4(0.00f, 0.00f, 0.00f, 0.52f);
	colors[ImGuiCol_TabHovered] = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
	colors[ImGuiCol_TabActive] = ImVec4(0.20f, 0.20f, 0.20f, 0.36f);
	colors[ImGuiCol_TabUnfocused] = ImVec4(0.00f, 0.00f, 0.00f, 0.52f);
	colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
	colors[ImGuiCol_DockingPreview] = ImVec4(0.33f, 0.67f, 0.86f, 1.00f);
	colors[ImGuiCol_DockingEmptyBg] = ImVec4(1.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotLines] = ImVec4(1.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotHistogram] = ImVec4(1.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_TableHeaderBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.52f);
	colors[ImGuiCol_TableBorderStrong] = ImVec4(0.00f, 0.00f, 0.00f, 0.52f);
	colors[ImGuiCol_TableBorderLight] = ImVec4(0.28f, 0.28f, 0.28f, 0.29f);
	colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.00f, 1.00f, 1.00f, 0.06f);
	colors[ImGuiCol_TextSelectedBg] = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
	colors[ImGuiCol_DragDropTarget] = ImVec4(0.33f, 0.67f, 0.86f, 1.00f);
	colors[ImGuiCol_NavHighlight] = ImVec4(1.00f, 0.00f, 0.00f, 1.00f);
	colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 0.00f, 0.00f, 0.70f);
	colors[ImGuiCol_NavWindowingDimBg] = ImVec4(1.00f, 0.00f, 0.00f, 0.20f);
	colors[ImGuiCol_ModalWindowDimBg] = ImVec4(1.00f, 0.00f, 0.00f, 0.35f);

	ImGuiStyle &style = ImGui::GetStyle();
	style.WindowPadding = ImVec2(8.00f, 8.00f);
	style.FramePadding = ImVec2(5.00f, 2.00f);
	style.CellPadding = ImVec2(6.00f, 6.00f);
	style.ItemSpacing = ImVec2(6.00f, 6.00f);
	style.ItemInnerSpacing = ImVec2(6.00f, 6.00f);
	style.TouchExtraPadding = ImVec2(0.00f, 0.00f);
	style.IndentSpacing = 25;
	style.ScrollbarSize = 15;
	style.GrabMinSize = 10;
	style.WindowBorderSize = 1;
	style.ChildBorderSize = 1;
	style.PopupBorderSize = 1;
	style.FrameBorderSize = 1;
	style.TabBorderSize = 1;
	style.WindowRounding = 7;
	style.ChildRounding = 4;
	style.FrameRounding = 3;
	style.PopupRounding = 4;
	style.ScrollbarRounding = 9;
	style.GrabRounding = 3;
	style.LogSliderDeadzone = 4;
	style.TabRounding = 4;
}

GodotImGui::GodotImGui() {
	singleton = this;

	// Makes the node process last
	set_process_priority(INT_MAX);
	set_process(true);

	set_mouse_filter(MouseFilter::MOUSE_FILTER_PASS);

	set_stretch(true);
	set_anchors_and_offsets_preset(Control::LayoutPreset::PRESET_FULL_RECT);
	set_process_unhandled_key_input(true);
	set_process_unhandled_input(true);

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImNodes::CreateContext();

	ImGuiIO &io = ImGui::GetIO();
	(void)io;
	io.IniFilename = nullptr;

	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

	ImGui::StyleColorsDark();

	viewport = memnew(SubViewport);
	viewport->set_clear_mode(SubViewport::CLEAR_MODE_NEVER);
	viewport->set_update_mode(SubViewport::UPDATE_DISABLED);
	viewport->set_transparent_background(true);

	add_child(viewport);

	_init_imgui();

	// scale_x, scale_y, translation_x, translation_y
	push_constant_buffer.resize(4);

	unsigned char *pixels;
	//pixels.resize(io.Fonts->TexWidth * io.Fonts->TexHeight * sizeof(float));
	int width, height;
	ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

	Vector<uint8_t> pixels_vec;
	pixels_vec.resize(width * height * 4);
	memcpy(pixels_vec.ptrw(), pixels, pixels_vec.size());

	Ref<Image> img = memnew(Image(width, height, false, Image::Format::FORMAT_RGBA8, pixels_vec));
	font_texture = ImageTexture::create_from_image(img);
	uint64_t rid = font_texture->get_rid().get_id();
	ImGui::GetIO().Fonts->SetTexID((void *)rid);
	embrace_the_darkness();
}

void GodotImGui::_begin_frame() {
	Size2 display_size = viewport->get_visible_rect().size;
	ImGui::GetIO().DisplaySize = ImVec2(display_size.x, display_size.y);
	ImGui::GetIO().DeltaTime = (float)get_process_delta_time();
	ImGui::NewFrame();
	ImGuizmo::SetDrawlist(ImGui::GetBackgroundDrawList());
	if (show_overlay) {
		_show_overlay();
	}
}

void GodotImGui::_show_overlay() {
	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
	const float PAD = 10.0f;
	const ImGuiViewport *viewport = ImGui::GetMainViewport();
	ImVec2 work_pos = viewport->WorkPos; // Use work area to avoid menu-bar/task-bar, if any!
	ImVec2 window_pos, window_pos_pivot;
	window_pos.x = work_pos.x + PAD;
	window_pos.y = work_pos.y + PAD;
	ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
	window_flags |= ImGuiWindowFlags_NoMove;
	ImGui::SetNextWindowBgAlpha(0.35);
	if (ImGui::Begin("Game Overlay", &show_overlay, window_flags)) {
		ImGui::Text("%.3f FPS", Performance::get_singleton()->get_monitor(Performance::TIME_FPS));
		ImGui::Text("%.3f ms frametime", Performance::get_singleton()->get_monitor(Performance::TIME_PROCESS) * 1000.0);
		ImGui::Separator();
		ImGui::Text("F1 - Show debug menu");
		ImGui::Text("F2 - Hide this overlay");
	}
	ImGui::End();
}

void GodotImGui::_recreate_framebuffer() {
	if (framebuffer.is_valid()) {
		framebuffer = RID();
	}
	RID vp_text = RenderingServer::get_singleton()->texture_get_rd_texture_rid(viewport->get_texture()->get_rid());
	Vector<RID> vp_text_a;
	vp_text_a.push_back(vp_text);
	framebuffer = RenderingDevice::get_singleton()->framebuffer_create(vp_text_a);
}

void GodotImGui::_end_frame() {
	ImGui::Render();
	_render_draw_data(ImGui::GetDrawData());
}

void GodotImGui::_render_draw_data(ImDrawData *p_draw_data) {
	RenderingDevice *rd = RenderingDevice::get_singleton();
	rd->draw_command_begin_label("ImGui");
	if (!rd) {
		return;
	}
	// Viewport likely changed size or just doesn't exist in the first place, we have to recreate it
	if (!rd->framebuffer_is_valid(framebuffer)) {
		_recreate_framebuffer();
	}

	int vertex_size = sizeof(ImDrawVert);

	push_constant_buffer.ptrw()[0] = 2.0 / p_draw_data->DisplaySize.x;
	push_constant_buffer.ptrw()[1] = 2.0 / p_draw_data->DisplaySize.y;
	push_constant_buffer.ptrw()[2] = -1.0 - (p_draw_data->DisplayPos.x * push_constant_buffer[0]);
	push_constant_buffer.ptrw()[3] = -1.0 - (p_draw_data->DisplayPos.y * push_constant_buffer[1]);

	if (index_buffer_size < p_draw_data->TotalIdxCount) {
		if (index_buffer.is_valid()) {
			rd->free(index_buffer);
		}
		index_buffer = rd->index_buffer_create(p_draw_data->TotalIdxCount, RenderingDevice::IndexBufferFormat::INDEX_BUFFER_FORMAT_UINT16);
		index_buffer_size = p_draw_data->TotalIdxCount;
	}
	if (vertex_buffer_size < p_draw_data->TotalIdxCount) {
		if (vertex_buffer.is_valid()) {
			rd->free(vertex_buffer);
		}
		vertex_buffer = rd->vertex_buffer_create(p_draw_data->TotalVtxCount * vertex_size);
		vertex_buffer_size = p_draw_data->TotalVtxCount;
	}

	used_textures.clear();

	// Send invalid textures to the shadow realm
	{
		Vector<uint64_t> to_erase;

		for (KeyValue<uint64_t, RID> kv : uniform_sets) {
			if (!rd->uniform_set_is_valid(kv.value)) {
				to_erase.push_back(kv.key);
			}
		}

		for (int key : to_erase) {
			uniform_sets.erase(key);
		}
	}

	if (p_draw_data->CmdListsCount > 0) {
		_setup_buffers(p_draw_data);
	}

	Vector<Color> clear_color;
	clear_color.push_back(Color(0.0, 0.0, 0.0, 0.0));

	RenderingDevice::DrawListID draw_list = rd->draw_list_begin(framebuffer,
			RenderingDevice::InitialAction::INITIAL_ACTION_CLEAR, RenderingDevice::FinalAction::FINAL_ACTION_READ,
			RenderingDevice::InitialAction::INITIAL_ACTION_CLEAR, RenderingDevice::FinalAction::FINAL_ACTION_READ,
			clear_color);
	rd->draw_list_bind_render_pipeline(draw_list, pipeline);
	rd->draw_list_set_push_constant(draw_list, push_constant_buffer.ptr(), push_constant_buffer.size() * 4);

	int index_offset = 0;
	int vertex_offset = 0;

	for (int i = 0; i < p_draw_data->CmdListsCount; i++) {
		ImDrawList *cmd_list = p_draw_data->CmdLists[i];

		for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++) {
			ImDrawCmd draw_cmd = cmd_list->CmdBuffer[cmd_i];
			if (draw_cmd.ElemCount == 0) {
				continue;
			}

			RID index_array = rd->index_array_create(index_buffer, draw_cmd.IdxOffset + index_offset, draw_cmd.ElemCount);

			uint64_t voff = (draw_cmd.VtxOffset + vertex_offset) * vertex_size;
			Vector<RID> src_buffers;
			src_buffers.resize(3);
			src_buffers.fill(vertex_buffer);
			Vector<uint64_t> vtx_offsets;
			vtx_offsets.resize(3);
			vtx_offsets.fill(voff);
			RID vertex_array = rd->vertex_array_create(cmd_list->VtxBuffer.Size, vertex_format, src_buffers, vtx_offsets);

			rd->draw_list_bind_uniform_set(draw_list, uniform_sets[(uint64_t)draw_cmd.GetTexID()], 0);
			rd->draw_list_bind_index_array(draw_list, index_array);
			rd->draw_list_bind_vertex_array(draw_list, vertex_array);

			Rect2 clip_rect = Rect2(
					draw_cmd.ClipRect.x,
					draw_cmd.ClipRect.y,
					draw_cmd.ClipRect.z - draw_cmd.ClipRect.x,
					draw_cmd.ClipRect.w - draw_cmd.ClipRect.y);

			clip_rect.position -= Vector2(p_draw_data->DisplayPos.x, p_draw_data->DisplayPos.y);

			rd->draw_list_enable_scissor(draw_list, clip_rect);

			rd->draw_list_draw(draw_list, true, 1);

			rd->free(index_array);
			rd->free(vertex_array);
		}
		index_offset += cmd_list->IdxBuffer.Size;
		vertex_offset += cmd_list->VtxBuffer.Size;
	}
	rd->draw_list_end();
	rd->draw_command_end_label();

	// clean up unused textures
	for (KeyValue<uint64_t, RID> kv : uniform_sets) {
		if (!used_textures.has(kv.key)) {
			rd->free(uniform_sets[kv.key]);
			uniform_sets.erase(kv.key);
		}
	}
}

void GodotImGui::_setup_buffers(ImDrawData *p_draw_data) {
	RenderingDevice *rd = RenderingDevice::get_singleton();

	int vertex_size = sizeof(ImDrawVert);

	PackedByteArray index_buffer_data;
	index_buffer_data.resize(p_draw_data->TotalIdxCount * sizeof(ushort));
	PackedByteArray vertex_buffer_data;
	vertex_buffer_data.resize(p_draw_data->TotalVtxCount * vertex_size);

	int vertex_offset = 0;
	int index_offset = 0;

	Vector<RenderingDevice::Uniform> uniform_array;
	uniform_array.resize(1);

	for (int i = 0; i < p_draw_data->CmdListsCount; i++) {
		ImDrawList *cmd_list = p_draw_data->CmdLists[i];

		int vertex_bytes = cmd_list->VtxBuffer.Size * vertex_size;
		memcpy(vertex_buffer_data.ptrw() + vertex_offset, cmd_list->VtxBuffer.Data, vertex_bytes);
		vertex_offset += vertex_bytes;

		int index_bytes = cmd_list->IdxBuffer.Size * sizeof(uint16_t);
		memcpy(index_buffer_data.ptrw() + index_offset, cmd_list->IdxBuffer.Data, index_bytes);
		index_offset += index_bytes;

		for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++) {
			ImDrawCmd draw_cmd = cmd_list->CmdBuffer[cmd_i];
			uint64_t tex_id = (uint64_t)draw_cmd.GetTexID();
			if (tex_id == 0) {
				continue;
			}

			used_textures.insert(tex_id);

			if (!uniform_sets.has(tex_id)) {
				RID tex_rid = RID::from_uint64(tex_id);
				tex_rid = RenderingServer::get_singleton()->texture_get_rd_texture_rid(tex_rid);
				RenderingDevice::Uniform uniform;
				uniform.binding = 0;
				uniform.uniform_type = RenderingDevice::UniformType::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE;
				uniform.append_id(sampler);
				uniform.append_id(tex_rid);
				uniform_array.ptrw()[0] = uniform;
				RID uniform_set = rd->uniform_set_create(uniform_array, shader, 0);
				uniform_sets[tex_id] = uniform_set;
			}
		}
	}

	rd->buffer_update(index_buffer, 0, index_buffer_data.size(), index_buffer_data.ptr());
	rd->buffer_update(vertex_buffer, 0, vertex_buffer_data.size(), vertex_buffer_data.ptr());
}
void GodotImGui::unhandled_key_input(const Ref<InputEvent> &p_event) {
	Ref<InputEventKey> ik = p_event;
	bool captured = false;

	ImGuiIO &io = ImGui::GetIO();

	if (ik.is_valid()) {
		ImGuiKey imk = _map_to_imgui_key(ik->get_keycode());
		io.AddKeyEvent(ImGuiKey::ImGuiMod_Ctrl, Input::get_singleton()->is_key_pressed(Key::CTRL));
		io.AddKeyEvent(ImGuiKey::ImGuiMod_Shift, Input::get_singleton()->is_key_pressed(Key::SHIFT));
		io.AddKeyEvent(ImGuiKey::ImGuiMod_Alt, Input::get_singleton()->is_key_pressed(Key::ALT));
		io.AddKeyEvent(ImGuiKey::ImGuiMod_Super, Input::get_singleton()->is_key_pressed(Key::META));
		if (imk != ImGuiKey::ImGuiKey_None) {
			io.AddKeyEvent(imk, ik->is_pressed());
			if (ik->get_unicode() != 0 && ik->is_pressed() && io.WantTextInput) {
				io.AddInputCharacter(ik->get_unicode());
			}
			captured = io.WantCaptureKeyboard || io.WantTextInput;
		}
	}

	if (captured) {
		accept_event();
	}
}

void GodotImGui::unhandled_input(const Ref<InputEvent> &p_event) {
	Ref<InputEventKey> kev = p_event;
	if (kev.is_valid()) {
		if (kev->is_pressed() && !kev->is_echo()) {
			if (kev->get_keycode() == Key::F1) {
				debug_active = !debug_active;
				Input::MouseMode mode = debug_active ? Input::MouseMode::MOUSE_MODE_VISIBLE : Input::MouseMode::MOUSE_MODE_CAPTURED;
				Input::get_singleton()->set_mouse_mode(mode);
				accept_event();
			} else if (kev->get_keycode() == Key::F2) {
				show_overlay = !show_overlay;
				accept_event();
			}
		}
	}
}

void GodotImGui::gui_input(const Ref<InputEvent> &p_event) {
	Ref<InputEventMouseMotion> mouse_mot_ev = p_event;
	ImGuiIO &io = ImGui::GetIO();
	if (mouse_mot_ev.is_valid()) {
		Vector2 mouse_pos = mouse_mot_ev->get_position();
		io.AddMousePosEvent(mouse_pos.x, mouse_pos.y);
	}

	bool captured = false;

	Ref<InputEventMouseButton> mouse_but_event = p_event;

	if (mouse_but_event.is_valid()) {
		switch (mouse_but_event->get_button_index()) {
			case MouseButton::LEFT: {
				io.AddMouseButtonEvent(ImGuiMouseButton_Left, mouse_but_event->is_pressed());
			} break;
			case MouseButton::RIGHT: {
				io.AddMouseButtonEvent(ImGuiMouseButton_Right, mouse_but_event->is_pressed());
			} break;
			case MouseButton::MIDDLE: {
				io.AddMouseButtonEvent(ImGuiMouseButton_Middle, mouse_but_event->is_pressed());
			} break;
			case MouseButton::WHEEL_UP: {
				io.AddMouseWheelEvent(0, mouse_but_event->get_factor());
			} break;
			case MouseButton::WHEEL_DOWN: {
				io.AddMouseWheelEvent(0, -mouse_but_event->get_factor());
			} break;
			case MouseButton::WHEEL_LEFT: {
				io.AddMouseWheelEvent(-mouse_but_event->get_factor(), 0);
			} break;
			case MouseButton::WHEEL_RIGHT: {
				io.AddMouseWheelEvent(mouse_but_event->get_factor(), 0);
			} break;
			case MouseButton::MB_XBUTTON1: {
				io.AddMouseButtonEvent(ImGuiMouseButton_Middle + 1, mouse_but_event->is_pressed());
			} break;
			case MouseButton::MB_XBUTTON2: {
				io.AddMouseButtonEvent(ImGuiMouseButton_Middle + 2, mouse_but_event->is_pressed());
			} break;
			default: {
			}
		}
		captured = io.WantCaptureMouse;
	}

	if (captured) {
		accept_event();
	}
}

// Not really necessary but a good placeholder so the IM_ASSERT() works
struct ImGui_ImplGodotRD_Data {
};

//-----------------------------------------------------------------------------
// SHADERS
//-----------------------------------------------------------------------------

// glsl_shader.vert, compiled with:
// # glslangValidator -V -x -o glsl_shader.vert.u32 glsl_shader.vert
/*
#version 450 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec4 aColor;
layout(push_constant) uniform uPushConstant { vec2 uScale; vec2 uTranslate; } pc;
out gl_PerVertex { vec4 gl_Position; };
layout(location = 0) out struct { vec4 Color; vec2 UV; } Out;
void main()
{
	Out.Color = aColor;
	Out.UV = aUV;
	gl_Position = vec4(aPos * pc.uScale + pc.uTranslate, 0, 1);
}
*/
static uint32_t __glsl_shader_vert_spv[] = {
	0x07230203, 0x00010000, 0x0008000b, 0x0000002e, 0x00000000, 0x00020011, 0x00000001, 0x0006000b,
	0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e, 0x00000000, 0x00000001,
	0x000a000f, 0x00000000, 0x00000004, 0x6e69616d, 0x00000000, 0x0000000b, 0x0000000f, 0x00000015,
	0x0000001b, 0x0000001c, 0x00030003, 0x00000002, 0x000001c2, 0x00040005, 0x00000004, 0x6e69616d,
	0x00000000, 0x00030005, 0x00000009, 0x00000000, 0x00050006, 0x00000009, 0x00000000, 0x6f6c6f43,
	0x00000072, 0x00040006, 0x00000009, 0x00000001, 0x00005655, 0x00030005, 0x0000000b, 0x0074754f,
	0x00040005, 0x0000000f, 0x6c6f4361, 0x0000726f, 0x00030005, 0x00000015, 0x00565561, 0x00060005,
	0x00000019, 0x505f6c67, 0x65567265, 0x78657472, 0x00000000, 0x00060006, 0x00000019, 0x00000000,
	0x505f6c67, 0x7469736f, 0x006e6f69, 0x00030005, 0x0000001b, 0x00000000, 0x00040005, 0x0000001c,
	0x736f5061, 0x00000000, 0x00060005, 0x0000001e, 0x73755075, 0x6e6f4368, 0x6e617473, 0x00000074,
	0x00050006, 0x0000001e, 0x00000000, 0x61635375, 0x0000656c, 0x00060006, 0x0000001e, 0x00000001,
	0x61725475, 0x616c736e, 0x00006574, 0x00030005, 0x00000020, 0x00006370, 0x00040047, 0x0000000b,
	0x0000001e, 0x00000000, 0x00040047, 0x0000000f, 0x0000001e, 0x00000002, 0x00040047, 0x00000015,
	0x0000001e, 0x00000001, 0x00050048, 0x00000019, 0x00000000, 0x0000000b, 0x00000000, 0x00030047,
	0x00000019, 0x00000002, 0x00040047, 0x0000001c, 0x0000001e, 0x00000000, 0x00050048, 0x0000001e,
	0x00000000, 0x00000023, 0x00000000, 0x00050048, 0x0000001e, 0x00000001, 0x00000023, 0x00000008,
	0x00030047, 0x0000001e, 0x00000002, 0x00020013, 0x00000002, 0x00030021, 0x00000003, 0x00000002,
	0x00030016, 0x00000006, 0x00000020, 0x00040017, 0x00000007, 0x00000006, 0x00000004, 0x00040017,
	0x00000008, 0x00000006, 0x00000002, 0x0004001e, 0x00000009, 0x00000007, 0x00000008, 0x00040020,
	0x0000000a, 0x00000003, 0x00000009, 0x0004003b, 0x0000000a, 0x0000000b, 0x00000003, 0x00040015,
	0x0000000c, 0x00000020, 0x00000001, 0x0004002b, 0x0000000c, 0x0000000d, 0x00000000, 0x00040020,
	0x0000000e, 0x00000001, 0x00000007, 0x0004003b, 0x0000000e, 0x0000000f, 0x00000001, 0x00040020,
	0x00000011, 0x00000003, 0x00000007, 0x0004002b, 0x0000000c, 0x00000013, 0x00000001, 0x00040020,
	0x00000014, 0x00000001, 0x00000008, 0x0004003b, 0x00000014, 0x00000015, 0x00000001, 0x00040020,
	0x00000017, 0x00000003, 0x00000008, 0x0003001e, 0x00000019, 0x00000007, 0x00040020, 0x0000001a,
	0x00000003, 0x00000019, 0x0004003b, 0x0000001a, 0x0000001b, 0x00000003, 0x0004003b, 0x00000014,
	0x0000001c, 0x00000001, 0x0004001e, 0x0000001e, 0x00000008, 0x00000008, 0x00040020, 0x0000001f,
	0x00000009, 0x0000001e, 0x0004003b, 0x0000001f, 0x00000020, 0x00000009, 0x00040020, 0x00000021,
	0x00000009, 0x00000008, 0x0004002b, 0x00000006, 0x00000028, 0x00000000, 0x0004002b, 0x00000006,
	0x00000029, 0x3f800000, 0x00050036, 0x00000002, 0x00000004, 0x00000000, 0x00000003, 0x000200f8,
	0x00000005, 0x0004003d, 0x00000007, 0x00000010, 0x0000000f, 0x00050041, 0x00000011, 0x00000012,
	0x0000000b, 0x0000000d, 0x0003003e, 0x00000012, 0x00000010, 0x0004003d, 0x00000008, 0x00000016,
	0x00000015, 0x00050041, 0x00000017, 0x00000018, 0x0000000b, 0x00000013, 0x0003003e, 0x00000018,
	0x00000016, 0x0004003d, 0x00000008, 0x0000001d, 0x0000001c, 0x00050041, 0x00000021, 0x00000022,
	0x00000020, 0x0000000d, 0x0004003d, 0x00000008, 0x00000023, 0x00000022, 0x00050085, 0x00000008,
	0x00000024, 0x0000001d, 0x00000023, 0x00050041, 0x00000021, 0x00000025, 0x00000020, 0x00000013,
	0x0004003d, 0x00000008, 0x00000026, 0x00000025, 0x00050081, 0x00000008, 0x00000027, 0x00000024,
	0x00000026, 0x00050051, 0x00000006, 0x0000002a, 0x00000027, 0x00000000, 0x00050051, 0x00000006,
	0x0000002b, 0x00000027, 0x00000001, 0x00070050, 0x00000007, 0x0000002c, 0x0000002a, 0x0000002b,
	0x00000028, 0x00000029, 0x00050041, 0x00000011, 0x0000002d, 0x0000001b, 0x0000000d, 0x0003003e,
	0x0000002d, 0x0000002c, 0x000100fd, 0x00010038
};

// glsl_shader.frag, compiled with:
// # glslangValidator -V -x -o glsl_shader.frag.u32 glsl_shader.frag
/*
#version 450 core
layout(location = 0) out vec4 fColor;
layout(set=0, binding=0) uniform sampler2D sTexture;
layout(location = 0) in struct { vec4 Color; vec2 UV; } In;
void main()
{
	fColor = In.Color * texture(sTexture, In.UV.st);
}
*/
static uint32_t __glsl_shader_frag_spv[] = {
	// 1111.13.0
	0x07230203, 0x00010000, 0x0008000b, 0x0000001e, 0x00000000, 0x00020011, 0x00000001, 0x0006000b,
	0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e, 0x00000000, 0x00000001,
	0x0007000f, 0x00000004, 0x00000004, 0x6e69616d, 0x00000000, 0x00000009, 0x0000000d, 0x00030010,
	0x00000004, 0x00000007, 0x00030003, 0x00000002, 0x000001c2, 0x00040005, 0x00000004, 0x6e69616d,
	0x00000000, 0x00040005, 0x00000009, 0x6c6f4366, 0x0000726f, 0x00030005, 0x0000000b, 0x00000000,
	0x00050006, 0x0000000b, 0x00000000, 0x6f6c6f43, 0x00000072, 0x00040006, 0x0000000b, 0x00000001,
	0x00005655, 0x00030005, 0x0000000d, 0x00006e49, 0x00050005, 0x00000016, 0x78655473, 0x65727574,
	0x00000000, 0x00040047, 0x00000009, 0x0000001e, 0x00000000, 0x00040047, 0x0000000d, 0x0000001e,
	0x00000000, 0x00040047, 0x00000016, 0x00000022, 0x00000000, 0x00040047, 0x00000016, 0x00000021,
	0x00000000, 0x00020013, 0x00000002, 0x00030021, 0x00000003, 0x00000002, 0x00030016, 0x00000006,
	0x00000020, 0x00040017, 0x00000007, 0x00000006, 0x00000004, 0x00040020, 0x00000008, 0x00000003,
	0x00000007, 0x0004003b, 0x00000008, 0x00000009, 0x00000003, 0x00040017, 0x0000000a, 0x00000006,
	0x00000002, 0x0004001e, 0x0000000b, 0x00000007, 0x0000000a, 0x00040020, 0x0000000c, 0x00000001,
	0x0000000b, 0x0004003b, 0x0000000c, 0x0000000d, 0x00000001, 0x00040015, 0x0000000e, 0x00000020,
	0x00000001, 0x0004002b, 0x0000000e, 0x0000000f, 0x00000000, 0x00040020, 0x00000010, 0x00000001,
	0x00000007, 0x00090019, 0x00000013, 0x00000006, 0x00000001, 0x00000000, 0x00000000, 0x00000000,
	0x00000001, 0x00000000, 0x0003001b, 0x00000014, 0x00000013, 0x00040020, 0x00000015, 0x00000000,
	0x00000014, 0x0004003b, 0x00000015, 0x00000016, 0x00000000, 0x0004002b, 0x0000000e, 0x00000018,
	0x00000001, 0x00040020, 0x00000019, 0x00000001, 0x0000000a, 0x00050036, 0x00000002, 0x00000004,
	0x00000000, 0x00000003, 0x000200f8, 0x00000005, 0x00050041, 0x00000010, 0x00000011, 0x0000000d,
	0x0000000f, 0x0004003d, 0x00000007, 0x00000012, 0x00000011, 0x0004003d, 0x00000014, 0x00000017,
	0x00000016, 0x00050041, 0x00000019, 0x0000001a, 0x0000000d, 0x00000018, 0x0004003d, 0x0000000a,
	0x0000001b, 0x0000001a, 0x00050057, 0x00000007, 0x0000001c, 0x00000017, 0x0000001b, 0x00050085,
	0x00000007, 0x0000001d, 0x00000012, 0x0000001c, 0x0003003e, 0x00000009, 0x0000001d, 0x000100fd,
	0x00010038
};

void GodotImGui::_init_imgui() {
	ImGuiIO &io = ImGui::GetIO();
	IM_ASSERT(io.BackendPlatformUserData == nullptr && "Already initialized a platform backend!");

	ImGui_ImplGodotRD_Data *bd = IM_NEW(ImGui_ImplGodotRD_Data)();
	io.BackendPlatformUserData = (void *)bd;
	io.BackendPlatformName = "imgui_impl_godotrd";
	io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;

	RenderingDevice *rd = RenderingDevice::get_singleton();
	if (!rd) {
		return;
	}
	int vtx_stride = sizeof(ImDrawVert);
	{
		Vector<RenderingDevice::VertexAttribute> vertex_attribs;
		RenderingDevice::VertexAttribute attrib_points;
		attrib_points.location = 0;
		attrib_points.format = RenderingDevice::DataFormat::DATA_FORMAT_R32G32_SFLOAT;
		attrib_points.stride = vtx_stride;
		attrib_points.offset = IM_OFFSETOF(ImDrawVert, pos);
		vertex_attribs.push_back(attrib_points);

		RenderingDevice::VertexAttribute attrib_uvs;
		attrib_uvs.location = 1;
		attrib_uvs.format = RenderingDevice::DataFormat::DATA_FORMAT_R32G32_SFLOAT;
		attrib_uvs.stride = vtx_stride;
		attrib_uvs.offset = IM_OFFSETOF(ImDrawVert, uv);
		vertex_attribs.push_back(attrib_uvs);

		RenderingDevice::VertexAttribute attrib_color;
		attrib_color.location = 2;
		attrib_color.format = RenderingDevice::DataFormat::DATA_FORMAT_R8G8B8A8_UNORM;
		attrib_color.stride = vtx_stride;
		attrib_color.offset = IM_OFFSETOF(ImDrawVert, col);
		vertex_attribs.push_back(attrib_color);

		vertex_format = rd->vertex_format_create(vertex_attribs);
	}

	RenderingDevice::PipelineColorBlendState blend_state = RenderingDevice::PipelineColorBlendState::create_blend(1);

	blend_state.blend_constant = Color(0.0, 0.0, 0.0, 0.0);
	blend_state.attachments.write[0].enable_blend = true;
	blend_state.attachments.write[0].src_color_blend_factor = RenderingDevice::BlendFactor::BLEND_FACTOR_SRC_ALPHA;
	blend_state.attachments.write[0].dst_color_blend_factor = RenderingDevice::BlendFactor::BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	blend_state.attachments.write[0].color_blend_op = RenderingDevice::BlendOperation::BLEND_OP_ADD;
	blend_state.attachments.write[0].src_alpha_blend_factor = RenderingDevice::BlendFactor::BLEND_FACTOR_ONE;
	blend_state.attachments.write[0].dst_alpha_blend_factor = RenderingDevice::BlendFactor::BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	blend_state.attachments.write[0].alpha_blend_op = RenderingDevice::BlendOperation::BLEND_OP_ADD;

	RenderingDevice::PipelineRasterizationState rasterization_state;
	rasterization_state.front_face = RenderingDevice::PolygonFrontFace::POLYGON_FRONT_FACE_COUNTER_CLOCKWISE;

	{
		Vector<RenderingDevice::ShaderStageSPIRVData> shader_stage_datas;

		RenderingDevice::ShaderStageSPIRVData vertex_shader;
		vertex_shader.shader_stage = RenderingDevice::ShaderStage::SHADER_STAGE_VERTEX;
		vertex_shader.spir_v.resize(sizeof(__glsl_shader_vert_spv));
		memcpy(vertex_shader.spir_v.ptrw(), __glsl_shader_vert_spv, sizeof(__glsl_shader_vert_spv));
		shader_stage_datas.push_back(vertex_shader);

		RenderingDevice::ShaderStageSPIRVData fragment_shader;
		fragment_shader.shader_stage = RenderingDevice::ShaderStage::SHADER_STAGE_FRAGMENT;
		fragment_shader.spir_v.resize(sizeof(__glsl_shader_frag_spv));
		memcpy(fragment_shader.spir_v.ptrw(), __glsl_shader_frag_spv, sizeof(__glsl_shader_frag_spv));
		shader_stage_datas.push_back(fragment_shader);

		shader = rd->shader_create_from_spirv(shader_stage_datas);
	}

	RenderingDevice::PipelineMultisampleState pmss;
	RenderingDevice::PipelineDepthStencilState pdss;
	_recreate_framebuffer();
	pipeline = rd->render_pipeline_create(
			shader,
			rd->framebuffer_get_format(framebuffer),
			vertex_format,
			RenderingDevice::RenderPrimitive::RENDER_PRIMITIVE_TRIANGLES,
			rasterization_state,
			pmss,
			pdss,
			blend_state);
	RenderingDevice::SamplerState sampler_state;
	sampler_state.min_filter = RenderingDevice::SamplerFilter::SAMPLER_FILTER_LINEAR;
	sampler_state.mag_filter = RenderingDevice::SamplerFilter::SAMPLER_FILTER_LINEAR;
	sampler_state.mip_filter = RenderingDevice::SamplerFilter::SAMPLER_FILTER_LINEAR;
	sampler_state.repeat_u = RenderingDevice::SamplerRepeatMode::SAMPLER_REPEAT_MODE_REPEAT;
	sampler_state.repeat_v = RenderingDevice::SamplerRepeatMode::SAMPLER_REPEAT_MODE_REPEAT;
	sampler_state.repeat_w = RenderingDevice::SamplerRepeatMode::SAMPLER_REPEAT_MODE_REPEAT;

	sampler = rd->sampler_create(sampler_state);
}

GodotImGui::~GodotImGui() {
	RenderingDevice *rd = RenderingDevice::get_singleton();
	if (pipeline.is_valid()) {
		rd->free(pipeline);
	}
	if (shader.is_valid()) {
		rd->free(shader);
	}
	if (sampler.is_valid()) {
		rd->free(sampler);
	}
	if (index_buffer.is_valid()) {
		rd->free(index_buffer);
	}
	if (vertex_buffer.is_valid()) {
		rd->free(vertex_buffer);
	}
	ImNodes::DestroyContext();
	ImGui::DestroyContext();
}

void GodotImGui::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			// Not sure why but putting this in the constructor doesn't really work
			SceneTree::get_singleton()->connect("process_frame", callable_mp(this, &GodotImGui::_begin_frame));
		} break;
		case NOTIFICATION_PROCESS: {
			if (debug_active) {
				if (ImGui::Begin("Game Debug", &debug_active)) {
					if (ImGui::BeginTabBar("DebugTabBar")) {
						if (ImGui::BeginTabItem("Objects")) {
							for (KeyValue<ObjectID, ObjectDebugInfo> kv : debug_status) {
								Object *obj = ObjectDB::get_instance(kv.key);
								Node *node = Object::cast_to<Node>(obj);
								String label_text;
								const String class_name = obj->get_class_name();
								if (node) {
									const String str = node->get_name();
									label_text = vformat("%s (%s) %d", str, class_name, static_cast<int64_t>(kv.key));
								} else {
									label_text = vformat("%s %d", class_name, static_cast<int64_t>(kv.key));
								}
								ImGui::Checkbox(label_text.utf8().ptr(), &(debug_status.getptr(kv.key)->debug_enabled));
							}
							ImGui::EndTabItem();
						}

						if (ImGui::BeginTabItem("Other")) {
							if (ImGui::Button("Toggle ImGui demo window")) {
								show_demo_window = !show_demo_window;
							}
							ImGui::AlignTextToFramePadding();
							ImGui::TextUnformatted("Timescale");
							ImGui::SameLine();
							if (ImGui::Button("1x")) {
								Engine::get_singleton()->set_time_scale(1.0);
							}
							ImGui::SameLine();
							if (ImGui::Button("0.5x")) {
								Engine::get_singleton()->set_time_scale(0.5);
							}
							ImGui::SameLine();
							if (ImGui::Button("0.25x")) {
								Engine::get_singleton()->set_time_scale(0.25);
							}
						}

						ImGui::EndTabBar();
					}
				}
				ImGui::End();
				if (show_demo_window) {
					ImGui::ShowDemoWindow(&show_demo_window);
				}
			}

			_end_frame();
		} break;
	}
}

bool GodotImGui::begin_debug_window(const Object *p_object) {
	ObjectDebugInfo *di_pointer = debug_status.getptr(p_object->get_instance_id());
	ERR_FAIL_COND_V_MSG(di_pointer == nullptr, false, "Tried to start a debug window for a non registered object");
	const Node *node = Object::cast_to<Node>(p_object);
	String window_title;
	String class_name = p_object->get_class_name();
	int64_t object_id = static_cast<int64_t>(p_object->get_instance_id());
	if (node) {
		window_title = vformat("%s (%s) - ID: %d", node->get_name(), class_name, object_id);
	} else {
		window_title = vformat("%s - ID: %d", class_name, object_id);
	}

	return ImGui::Begin(window_title.utf8().ptr(), &(di_pointer->debug_enabled));
}

bool GodotImGui::is_debug_enabled(const Object *p_object) const {
	const ObjectDebugInfo *di_pointer = debug_status.getptr(p_object->get_instance_id());
	ERR_FAIL_COND_V_MSG(di_pointer == nullptr, false, "Tried to start a debug status info for a non registered object");
	return di_pointer->debug_enabled;
}

void GodotImGui::set_enable_overlay(bool p_enable) {
	show_overlay = p_enable;
}

ImGuiKey GodotImGui::_map_to_imgui_key(const Key &p_key) {
	ImGuiKey imgui_key = ImGuiKey_None;
	if (p_key >= Key::A && p_key <= Key::Z) {
		imgui_key = (ImGuiKey)(ImGuiKey_A + (p_key - Key::A));
	} else if (p_key >= Key::KEY_0 && p_key <= Key::KEY_9) {
		imgui_key = (ImGuiKey)(ImGuiKey_0 + (p_key - Key::KEY_0));
	} else if (p_key >= Key::KP_0 && p_key <= Key::KP_9) {
		imgui_key = (ImGuiKey)(ImGuiKey_Keypad0 + (p_key - Key::KP_0));
	} else if (p_key >= Key::F1 && p_key <= Key::F12) {
		imgui_key = (ImGuiKey)(ImGuiKey_F1 + (p_key - Key::F1));
	} else {
		switch (p_key) {
			case Key::LEFT:
				imgui_key = ImGuiKey_LeftArrow;
				break;
			case Key::UP:
				imgui_key = ImGuiKey_UpArrow;
				break;
			case Key::RIGHT:
				imgui_key = ImGuiKey_RightArrow;
				break;
			case Key::DOWN:
				imgui_key = ImGuiKey_DownArrow;
				break;
			case Key::TAB:
				imgui_key = ImGuiKey_Tab;
				break;
			case Key::KP_PERIOD:
				imgui_key = ImGuiKey_KeypadDecimal;
				break;
			case Key::KP_ADD:
				imgui_key = ImGuiKey_KeypadAdd;
				break;
			case Key::KP_SUBTRACT:
				imgui_key = ImGuiKey_KeypadSubtract;
				break;
			case Key::KP_ENTER:
				imgui_key = ImGuiKey_KeypadEnter;
				break;
			case Key::KP_DIVIDE:
				imgui_key = ImGuiKey_KeypadDivide;
				break;
			case Key::KP_MULTIPLY:
				imgui_key = ImGuiKey_KeypadMultiply;
				break;
			case Key::PAGEUP:
				imgui_key = ImGuiKey_PageUp;
				break;
			case Key::PAGEDOWN:
				imgui_key = ImGuiKey_PageDown;
				break;
			case Key::SHIFT:
				imgui_key = ImGuiKey_LeftShift;
				break;
			case Key::CTRL:
				imgui_key = ImGuiKey_LeftCtrl;
				break;
			case Key::META:
				imgui_key = ImGuiKey_LeftSuper;
				break;
			case Key::ALT:
				imgui_key = ImGuiKey_LeftAlt;
				break;
			case Key::CAPSLOCK:
				imgui_key = ImGuiKey_CapsLock;
				break;
			case Key::NUMLOCK:
				imgui_key = ImGuiKey_NumLock;
				break;
			case Key::APOSTROPHE:
				imgui_key = ImGuiKey_Apostrophe;
				break;
			case Key::COMMA:
				imgui_key = ImGuiKey_Comma;
				break;
			case Key::MINUS:
				imgui_key = ImGuiKey_Minus;
				break;
			case Key::PERIOD:
				imgui_key = ImGuiKey_Period;
				break;
			case Key::SLASH:
				imgui_key = ImGuiKey_Slash;
				break;
			case Key::SEMICOLON:
				imgui_key = ImGuiKey_Semicolon;
				break;
			case Key::EQUAL:
				imgui_key = ImGuiKey_Equal;
				break;
			case Key::BRACKETLEFT:
				imgui_key = ImGuiKey_LeftBracket;
				break;
			case Key::BACKSLASH:
				imgui_key = ImGuiKey_Backslash;
				break;
			case Key::BRACKETRIGHT:
				imgui_key = ImGuiKey_RightBracket;
				break;
			case Key::PAUSE:
				imgui_key = ImGuiKey_Pause;
				break;
			case Key::HOME:
				imgui_key = ImGuiKey_Home;
				break;
			case Key::END:
				imgui_key = ImGuiKey_End;
				break;
			case Key::BACKSPACE:
				imgui_key = ImGuiKey_Backspace;
				break;
			case Key::ENTER:
				imgui_key = ImGuiKey_Enter;
				break;
			case Key::SPACE:
				imgui_key = ImGuiKey_Space;
				break;
			case Key::ESCAPE:
				imgui_key = ImGuiKey_Escape;
				break;
			default: {
			};
		}
	}

	return imgui_key;
}

void GodotImGui::register_debug_object(const ObjectID &p_object_id) {
	ERR_FAIL_COND_MSG(debug_status.has(p_object_id), "GAME: Error registering debug object, already registered");
	const ObjectDebugInfo debug_data;
	debug_status.insert(p_object_id, debug_data);
}

void GodotImGui::unregister_debug_object(const ObjectID &p_object_id) {
	ERR_FAIL_COND_MSG(!debug_status.has(p_object_id), "GAME: Error unregistering debug object, object is not registered");
	debug_status.erase(p_object_id);
}

bool GodotImGui::is_debugging_enabled(const ObjectID &p_object_id) const {
	const ObjectDebugInfo *debug_data = debug_status.getptr(p_object_id);
	ERR_FAIL_COND_V_MSG(debug_data == nullptr, false, "Error getting debug status for object, object is not registered");
	return debug_data->debug_enabled;
}
