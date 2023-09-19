#ifndef INPUT_GLYPH_DEMO_TOOL_H
#define INPUT_GLYPH_DEMO_TOOL_H

#include "modules/modules_enabled.gen.h"

#if defined(MODULE_STEAMWORKS_ENABLED) && defined(MODULE_INPUT_GLYPHS_ENABLED)

#include "modules/imgui/godot_imgui.h"
#include "modules/input_glyphs/input_glyphs.h"
#include "modules/input_glyphs/input_glyphs_constants.h"
#include "modules/input_glyphs/input_glyphs_singleton.h"
#include "modules/steamworks/steam_input.h"
#include <iterator>

class InputGlyphDemoTool : public GodotImGuiTool {
	GDCLASS(InputGlyphDemoTool, GodotImGuiTool);
	struct GlyphDebugInfo {
		InputGlyphsConstants::InputOrigin origin;
		Ref<Texture2D> texture;
		int style;
	};

	int input_type_idx = InputGlyphsConstants::XBOX_ONE_CONTROLLER;
	int glyph_size = InputGlyphSize::GLYPH_SIZE_SMALL;

	// 3 themes, knockout light and dark
	// we also add 3 spaces per-theme for the different variants of ABXY
	// (GLYPH_STYLE_NEUTRAL_COLOR_ABXY, GLYPH_STYLE_SOLID_ABXY, GLYPH_STYLE_NEUTRAL_COLOR_ABXY + GLYPH_STYLE_SOLID_ABXY)
	static const int GLYPH_COUNT = InputGlyphsConstants::INPUT_ORIGIN_COUNT + (4 * 3);
	static const int THEME_COUNT = 3;

	const char *THEME_NAMES[THEME_COUNT] = {
		"Knockout",
		"Light",
		"Dark",
	};

	GlyphDebugInfo glyph_infos[THEME_COUNT][GLYPH_COUNT];
	bool has_glyphs = false;
	void draw_glyphs(int p_theme_i);
	void reload_glyphs();
	bool open = false;

public:
	bool get_open() const;
	void set_open(bool p_open);

	virtual void draw_ui() override;
	virtual String get_name() const override;
};

#endif // defined(MODULE_STEAMWORKS_ENABLED) && defined(MODULE_INPUT_GLYPHS_ENABLED)
#endif // INPUT_GLYPH_DEMO_TOOL_H
