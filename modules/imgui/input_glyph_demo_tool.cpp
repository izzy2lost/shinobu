#include "modules/modules_enabled.gen.h"

#if defined(MODULE_INPUT_GLYPHS_ENABLED)

#include "godot_imgui.h"
#include "input_glyph_demo_tool.h"

void InputGlyphDemoTool::reload_glyphs() {
	Ref<InputGlyphsSource> source = InputGlyphsSource::create();

	InputGlyphsConstants::InputType input_type = (InputGlyphsConstants::InputType)input_type_idx;

	for (int theme_i = 0; theme_i < THEME_COUNT; theme_i++) {
		int base_style = theme_i;
		const int ABXY_STYLES[4] = {
			base_style,
			base_style | InputGlyphStyle::GLYPH_STYLE_NEUTRAL_COLOR_ABXY,
			base_style | InputGlyphStyle::GLYPH_STYLE_SOLID_ABXY,
			base_style | InputGlyphStyle::GLYPH_STYLE_SOLID_ABXY | InputGlyphStyle::GLYPH_STYLE_NEUTRAL_COLOR_ABXY,
		};
		// First get the different ABXY variations
		for (int i = 0; i < 4 * 4; i++) {
			int style = ABXY_STYLES[i / 4];
			GlyphDebugInfo info;
			info.origin = (InputGlyphsConstants::InputOrigin)(i % 4);
			info.texture = source->get_input_glyph(input_type, info.origin, style, (InputGlyphSize)glyph_size);
			info.style = style;
			glyph_infos[theme_i][i] = info;
		}

		// Now do the rest
		for (int i = 4; i < InputGlyphsConstants::InputOrigin::INPUT_ORIGIN_COUNT; i++) {
			GlyphDebugInfo info;
			info.origin = (InputGlyphsConstants::InputOrigin)i;
			info.texture = source->get_input_glyph(input_type, info.origin, base_style, (InputGlyphSize)glyph_size);
			info.style = base_style;
			glyph_infos[theme_i][(4 * 3) + i] = info;
		}
	}

	has_glyphs = true;
}

void InputGlyphDemoTool::draw_glyphs(int p_theme_i) {
	for (int i = 0; i < GLYPH_COUNT; i++) {
		if (i % 4 != 0) {
			ImGui::SameLine();
		}
		Ref<Texture2D> tex = glyph_infos[p_theme_i][i].texture;
		Vector2i size = Vector2i(32, 32);
		if (tex.is_valid() && tex->get_size() > Vector2i()) {
			size = tex->get_size();
		}
		GodotImGui::ImImage(tex, size);
		if (!tex.is_valid()) {
			continue;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextUnformatted(InputGlyphsConstants::_debug_input_origin_names[glyph_infos[p_theme_i][i].origin]);
			String path = tex->get_meta("glyph_path", "");
			path = path.get_file();
			ImGui::TextUnformatted(path.utf8().get_data());
			ImGui::Separator();

			ImGui::Text("%dx%d", tex->get_width(), tex->get_height());
			int style = glyph_infos[p_theme_i][i].style;
			String theme_name = String(THEME_NAMES[style & 0b11]).to_upper();
			ImGui::Text("GLYPH_STYLE_%s", theme_name.utf8().get_data());

			if (style & GLYPH_STYLE_NEUTRAL_COLOR_ABXY) {
				ImGui::TextUnformatted("GLYPH_STYLE_NEUTRAL_COLOR_ABXY");
			}
			if (style & GLYPH_STYLE_SOLID_ABXY) {
				ImGui::TextUnformatted("GLYPH_STYLE_SOLID_ABXY");
			}
			ImGui::Text("%X", style);

			ImGui::EndTooltip();
		}
	}
}

void InputGlyphDemoTool::draw_ui() {
	if (!has_glyphs) {
		reload_glyphs();
	}

	static const char *input_type_names[] = {
		"Unknown",
		"Steam Controller",
		"Xbox 360 Controller",
		"Xbox One Controller",
		"Generic XInput Controller",
		"PS3 Controller",
		"PS4 Controller",
		"PS5 Controller",
		"Switch Pro Controller",
		"Steam Deck Controller",
		"Keyboard",
	};

	static_assert(std::size(input_type_names) == InputGlyphsConstants::InputType::INPUT_TYPE_MAX);

	static const char *input_size_names[] = {
		"Small - 32x32",
		"Medium - 128x128",
		"Large - 256x256",
	};

	static_assert(std::size(input_size_names) == InputGlyphSize::GLYPH_SIZE_MAX);

	TypedArray<int> connected = Input::get_singleton()->get_connected_joypads();
	for (int i = 0; i < connected.size(); i++) {
		int number = connected[i];
		ImGui::Text("%d %d %s", i, number, Input::get_singleton()->get_joy_name(number).utf8().get_data());
		InputGlyphsConstants::InputType it = InputGlyphsSingleton::get_singleton()->glyph_source->identify_joy(number);
		ImGui::Text("%d %s", it, input_type_names[it]);
		if (Input::get_singleton()->is_joy_button_pressed(number, JoyButton::A)) {
			ImGui::SameLine();
			ImGui::TextUnformatted("A PRESSED!");
		}
		ImGui::Separator();
	}
	if (ImGui::BeginCombo("Input Type", input_type_names[input_type_idx])) {
		for (int i = 0; i < InputGlyphsConstants::InputType::INPUT_TYPE_MAX; i++) {
			if (ImGui::Selectable(input_type_names[i], i == input_type_idx)) {
				input_type_idx = i;
				reload_glyphs();
			}
		}
		ImGui::EndCombo();
	}
	if (ImGui::BeginCombo("Glyph Size", input_size_names[glyph_size])) {
		for (int i = 0; i < InputGlyphSize::GLYPH_SIZE_MAX; i++) {
			if (ImGui::Selectable(input_size_names[i], i == glyph_size)) {
				glyph_size = i;
				reload_glyphs();
			}
		}
		ImGui::EndCombo();
	}
	for (int i = 0; i < THEME_COUNT; i++) {
		if (ImGui::CollapsingHeader(THEME_NAMES[i])) {
			draw_glyphs(i);
		}
	}
};

bool InputGlyphDemoTool::get_open() const {
	return open;
}

void InputGlyphDemoTool::set_open(bool p_open) {
	open = p_open;
}

String InputGlyphDemoTool::get_name() const {
	return "Glyph Demo Tool";
}

#endif