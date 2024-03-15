/**************************************************************************/
/*  console.h                                                             */
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

#ifndef CONSOLE_H
#define CONSOLE_H

#include "scene/gui/line_edit.h"
#include "scene/gui/panel_container.h"
#include "scene/gui/rich_text_label.h"

class HBConsole : public PanelContainer {
	RichTextLabel *label = nullptr;
	LineEdit *text_input = nullptr;
	PrintHandlerList phl;
	ErrorHandlerList ehl;
	static void _handle_print(void *p_this, const String &p_string, bool p_error, bool p_rich);
	static void _handle_error(void *p_self, const char *p_func, const char *p_file, int p_line, const char *p_error, const char *p_errorexp, bool p_editor_notify, ErrorHandlerType p_type);

	void _handle_print_impl(const String &p_string, bool p_error, bool p_rich);
	void _handle_error_impl(const String &p_func, const String &p_file, int p_line, const String &p_error, const String &p_errorexp, int p_type);
	void _on_text_changed(const String &p_text);
	void _on_text_submitted(const String &p_text);
	PopupMenu *autocomplete_menu = nullptr;
	void _on_autocomplete_index_pressed(int p_idx);

	static constexpr int HISTORY_MAX = 16;
	LocalVector<String> history;
	uint32_t history_index = 0;

	bool set_text_from_history(int p_history_delta);

public:
	virtual void input(const Ref<InputEvent> &p_event) override;
	virtual void gui_input(const Ref<InputEvent> &p_event) override;
	HBConsole();
	~HBConsole();
};

#endif // CONSOLE_H
