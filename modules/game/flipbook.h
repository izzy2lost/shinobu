/**************************************************************************/
/*  flipbook.h                                                            */
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

#ifndef FLIPBOOK_H
#define FLIPBOOK_H

#include "scene/resources/texture.h"

class HBFlipbook : public Resource {
	GDCLASS(HBFlipbook, Resource);
	Ref<Texture2D> texture;

	struct FlipbookFrame {
		Rect2 region;
		Rect2 margin;
	};

	Vector<FlipbookFrame> frames;
	TypedArray<Rect2> _get_frames();
	void _set_frames(TypedArray<Rect2> p_frames);

protected:
	static void _bind_methods();

public:
	void add_frame(Rect2 p_region, Rect2 p_margin);

	PackedVector2Array calculate_shader_data() const;

	Ref<Texture2D> get_texture() const;
	void set_texture(const Ref<Texture2D> &p_texture);
	int get_frame_count() const;
};

#endif // FLIPBOOK_H
