#ifndef GAME_TOOLS_THEME_H
#define GAME_TOOLS_THEME_H

#include "modules/svg/image_loader_svg.h"
#include "scene/resources/texture.h"
#include "scene/resources/theme.h"
#include "tool_icons/game_tools_theme_icons.gen.h"

// See also `editor_generate_icon()` in `editor/editor_themes.cpp`.
namespace GameToolsTheme {
static Theme *theme = nullptr;
static Ref<ImageTexture> generate_icon(int p_index) {
	Ref<Image> img = memnew(Image);

	const float scale = 1.0f;

#ifdef MODULE_SVG_ENABLED
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
} //namespace GameToolsTheme

#endif // GAME_TOOLS_THEME_H
