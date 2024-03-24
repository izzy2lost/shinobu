/**************************************************************************/
/*  flipbook.cpp                                                          */
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

#include "flipbook.h"

TypedArray<Rect2> HBFlipbook::_get_frames() {
	TypedArray<Rect2> out;
	out.resize(frames.size() * 2);
	for (int i = 0; i < frames.size(); i++) {
		const FlipbookFrame &frame = frames[i];
		out[i * 2] = frame.region;
		out[i * 2 + 1] = frame.margin;
	}
	return out;
}

void HBFlipbook::_set_frames(TypedArray<Rect2> p_frames) {
	ERR_FAIL_COND_MSG(p_frames.size() % 2 != 0, "Frames must be a multiple of 2");
	print_line("SET FRAMES", p_frames);
	frames.clear();
	frames.resize(p_frames.size() / 2);
	FlipbookFrame *frw = frames.ptrw();
	for (int i = 0; i < frames.size(); i++) {
		frw[i].region = p_frames[i * 2];
		frw[i].margin = p_frames[i * 2 + 1];
	}
}

void HBFlipbook::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_frame", "region", "margin"), &HBFlipbook::add_frame);
	ClassDB::bind_method(D_METHOD("calculate_shader_data"), &HBFlipbook::calculate_shader_data);

	ClassDB::bind_method(D_METHOD("get_frame_count"), &HBFlipbook::get_frame_count);

	ClassDB::bind_method(D_METHOD("get_texture"), &HBFlipbook::get_texture);
	ClassDB::bind_method(D_METHOD("set_texture", "texture"), &HBFlipbook::set_texture);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "texture", PROPERTY_HINT_RESOURCE_TYPE, "Texture2D"), "set_texture", "get_texture");

	ClassDB::bind_method(D_METHOD("_set_frames", "frames"), &HBFlipbook::_set_frames);
	ClassDB::bind_method(D_METHOD("_get_frames"), &HBFlipbook::_get_frames);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "frames", PROPERTY_HINT_ARRAY_TYPE, "Rect2", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_INTERNAL), "_set_frames", "_get_frames");
}

void HBFlipbook::add_frame(Rect2 p_region, Rect2 p_margin) {
	frames.push_back(FlipbookFrame{
			.region = p_region,
			.margin = p_margin });
}

PackedVector2Array HBFlipbook::calculate_shader_data() const {
	ERR_FAIL_COND_V_MSG(!texture.is_valid(), PackedVector2Array(), "A texture is needed to calculate shader data!");
	PackedVector2Array out;
	out.resize(frames.size() * 4);
	Vector2 *out_w = out.ptrw();
	Vector2 tex_size = texture->get_size();
	for (int i = 0; i < frames.size(); i++) {
		const FlipbookFrame &frame = frames[i];
		Vector2 original_size = frame.region.size + frame.margin.size;
		Vector2 org = (frame.margin.position) / original_size;
		Vector2 siz = (frame.region.size) / original_size;

		out_w[i * 4 + 0] = org;
		out_w[i * 4 + 1] = siz;
		out_w[i * 4 + 2] = frame.region.position / tex_size;
		out_w[i * 4 + 3] = frame.region.size / tex_size;
	}
	return out;
}

Ref<Texture2D> HBFlipbook::get_texture() const { return texture; }

void HBFlipbook::set_texture(const Ref<Texture2D> &p_texture) { texture = p_texture; }

int HBFlipbook::get_frame_count() const {
	return frames.size();
}
