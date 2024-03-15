/**************************************************************************/
/*  game_tools_theme.h                                                    */
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

#ifndef GAME_TOOLS_THEME_H
#define GAME_TOOLS_THEME_H

#include "modules/svg/image_loader_svg.h"
#include "scene/resources/image_texture.h"
#include "scene/resources/theme.h"
#include "tool_icons/game_tools_theme_icons.gen.h"

// See also `editor_generate_icon()` in `editor/editor_themes.cpp`.
class GameToolsTheme {
	static Theme *theme;

public:
	static Ref<ImageTexture> generate_icon(int p_index) {
		Ref<Image> img = memnew(Image);

#ifdef MODULE_SVG_ENABLED
		const float scale = 1.0f;
		// Upsample icon generation only if the scale isn't an integer multiplier.
		// Generating upsampled icons is slower, and the benefit is hardly visible
		// with integer scales.
		const bool upsample = !Math::is_equal_approx(Math::round(scale), scale);
		ImageLoaderSVG img_loader;
		Error err = img_loader.create_image_from_string(img, game_tools_theme_icons_sources[p_index], scale, upsample, HashMap<Color, Color>());
		ERR_FAIL_COND_V_MSG(err != OK, Ref<ImageTexture>(), "Failed generating icon, unsupported or invalid SVG data in game tools theme.");
#endif
		return ImageTexture::create_from_image(img);
	}
	static Ref<Theme> generate_theme() {
		// No need to generate this twice
		if (theme) {
			return theme;
		}
		Ref<Theme> out_theme;
		out_theme.instantiate();

		for (int i = 0; i < game_tools_theme_icons_count; i++) {
			out_theme->set_icon(game_tools_theme_icons_names[i], "GameTools", generate_icon(i));
		}
		theme = out_theme.ptr();
		return out_theme;
	}
};

#endif // GAME_TOOLS_THEME_H
