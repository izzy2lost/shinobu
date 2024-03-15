/**************************************************************************/
/*  console.cpp                                                           */
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

#include "console.h"
#include "console_system.h"
#include "core/variant/variant_utility.h"
#include "scene/gui/box_container.h"
#include "scene/main/viewport.h"

HBConsole::HBConsole() {
	hide();
	set_anchors_and_offsets_preset(LayoutPreset::PRESET_FULL_RECT);
	set_anchor(SIDE_BOTTOM, 0.35f);
	set_process_input(true);

	VBoxContainer *container = memnew(VBoxContainer);

	add_child(container);
	container->set_anchors_and_offsets_preset(LayoutPreset::PRESET_FULL_RECT);

	label = memnew(RichTextLabel);
	container->add_child(label);

	label->set_scroll_active(true);
	label->set_v_size_flags(SizeFlags::SIZE_EXPAND_FILL);
	label->set_scroll_follow(true);
	label->set_use_bbcode(true);
	label->set_threaded(true);

	text_input = memnew(LineEdit);
	container->add_child(text_input);
	text_input->connect("text_changed", callable_mp(this, &HBConsole::_on_text_changed));
	text_input->connect("text_submitted", callable_mp(this, &HBConsole::_on_text_submitted));

	phl.printfunc = &HBConsole::_handle_print;
	phl.userdata = this;
	add_print_handler(&phl);

	ehl.userdata = this;
	ehl.errfunc = &HBConsole::_handle_error;
	add_error_handler(&ehl);

	autocomplete_menu = memnew(PopupMenu);
	autocomplete_menu->set_exclusive(false);
	add_child(autocomplete_menu);
	autocomplete_menu->hide();
	autocomplete_menu->connect("index_pressed", callable_mp(this, &HBConsole::_on_autocomplete_index_pressed));

	set_process_mode(PROCESS_MODE_ALWAYS);
}

HBConsole::~HBConsole() {
	remove_print_handler(&phl);
	remove_error_handler(&ehl);
}

void HBConsole::_handle_print(void *p_this, const String &p_string, bool p_error, bool p_rich) {
	HBConsole *console = static_cast<HBConsole *>(p_this);
	callable_mp(console, &HBConsole::_handle_print_impl).bind(p_string, p_error, p_rich).call_deferred();
}

void HBConsole::_handle_error(void *p_this, const char *p_func, const char *p_file, int p_line, const char *p_error, const char *p_errorexp, bool p_editor_notify, ErrorHandlerType p_type) {
	HBConsole *console = static_cast<HBConsole *>(p_this);
	callable_mp(console, &HBConsole::_handle_error_impl).bind(p_func, p_file, p_line, p_error, p_errorexp, p_type).call_deferred();
}

void HBConsole::_handle_print_impl(const String &p_string, bool p_error, bool p_rich) {
	if (p_error) {
		label->push_color(Color(1.0f, 0.0f, 0.0f));
	}
	if (p_rich) {
		label->append_text(p_string);
	} else {
		label->add_text(p_string);
	}
	if (p_error) {
		label->pop();
	}
	label->add_newline();
}

void HBConsole::_handle_error_impl(const String &p_func, const String &p_file, int p_line, const String &p_error, const String &p_errorexp, int p_type) {
	Color col;
	ErrorHandlerType p_err_type = (ErrorHandlerType)p_type;
	switch (p_err_type) {
		case ERR_HANDLER_ERROR: {
			label->push_color(Color(1.0f, 0.0f, 0.0f));
			label->push_bold();
			label->add_text("ERROR: ");
			label->pop();
		} break;
		case ERR_HANDLER_WARNING: {
			label->push_color(Color(1.0f, 1.0f, 0.0f));
			label->push_bold();
			label->add_text("WARNING: ");
			label->pop();
		} break;
		case ERR_HANDLER_SCRIPT: {
			label->push_color(Color(1.0f, 0.0f, 1.0f));
			label->push_bold();
			label->add_text("SCRIPT ERROR: ");
			label->pop();
		} break;
		case ERR_HANDLER_SHADER: {
			label->push_color(Color(1.0f, 1.0f, 0.5f));
			label->push_bold();
			label->add_text("SHADER ERROR: ");
			label->pop();
		} break;
	}
	if (p_errorexp.size() > 0) {
		label->add_text("(");
		label->add_text(p_errorexp);
		label->add_text(")");
	}

	label->add_text(p_error);
	label->pop();
	label->add_newline();
	label->push_color(Color(0.75f, 0.75f, 0.75f));
	label->add_text(vformat("\t at: %s (%s:%d)", p_func, p_file, p_line));
	label->pop();
	label->add_newline();
}

void HBConsole::_on_text_changed(const String &p_text) {
	static constexpr int MAX_MATCHES = 32;
	static ConsoleSystem::CTextMatch matches[MAX_MATCHES];
	int match_count = ConsoleSystem::get_singleton()->token_match(p_text, matches, MAX_MATCHES);
	autocomplete_menu->clear();
	for (int i = 0; i < match_count; i++) {
		if (matches[i].data->get_token_type() == CTokenData::COMMAND) {
			autocomplete_menu->add_item(matches[i].data->get_name());
			autocomplete_menu->set_item_metadata(i, matches[i].data->get_name());
		} else {
			autocomplete_menu->add_item(matches[i].data->get_name() + " " + String(matches[i].data->get_value()));
			autocomplete_menu->set_item_metadata(i, matches[i].data->get_name() + " ");
		}
	}

	Vector2 start = text_input->get_global_position();
	start += Vector2(0, text_input->get_size().y);

	if (autocomplete_menu->get_item_count() == 0) {
		autocomplete_menu->hide();
	} else {
		autocomplete_menu->set_flag(Window::FLAG_NO_FOCUS, true);
		autocomplete_menu->popup(Rect2i(start, Vector2(0, text_input->get_size().y)));
		autocomplete_menu->set_flag(Window::FLAG_NO_FOCUS, false);
	}
}

void HBConsole::_on_text_submitted(const String &p_text) {
	history.push_back(p_text);
	history_index = -1;
	if (history.size() > HISTORY_MAX) {
		history.remove_at(0);
	}
	ConsoleSystem::get_singleton()->parse_user_command(p_text);
	text_input->clear();
}

void HBConsole::_on_autocomplete_index_pressed(int p_idx) {
	text_input->set_text(String(autocomplete_menu->get_item_metadata(p_idx)) + " ");
	text_input->grab_focus();
	text_input->set_caret_column(text_input->get_text().size());
}

bool HBConsole::set_text_from_history(int p_history_delta) {
	uint32_t prev_history_index = history_index;
	history_index += p_history_delta;
	history_index = CLAMP(history_index, 0u, MAX(history.size() - 1, 0u));

	if (prev_history_index != history_index) {
		if (history_index < history.size()) {
			text_input->set_text(history[history_index]);
			text_input->set_caret_column(text_input->get_text().size());
			return true;
		}
	}
	return false;
}

void HBConsole::input(const Ref<InputEvent> &p_event) {
	if (p_event->is_action_pressed("toggle_console")) {
		set_visible(!is_visible());
		get_viewport()->set_input_as_handled();
		if (is_visible()) {
			text_input->grab_focus();
		}
		get_tree()->set_pause(is_visible());
	}
	if (p_event->is_action_pressed("ui_text_caret_down")) {
		if (autocomplete_menu->is_visible() && !autocomplete_menu->has_focus()) {
			autocomplete_menu->grab_focus();
			autocomplete_menu->set_focused_item(0);
			accept_event();
			return;
		}
		if (set_text_from_history(1)) {
			accept_event();
		}
	}
	if (p_event->is_action_pressed("ui_text_caret_up")) {
		if (set_text_from_history(-1)) {
			accept_event();
		}
	}
}

void HBConsole::gui_input(const Ref<InputEvent> &p_event) {
}
