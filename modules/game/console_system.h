/**************************************************************************/
/*  console_system.h                                                      */
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

#ifndef CVAR_H
#define CVAR_H

#include "core/object/class_db.h"
#include "core/object/object.h"
#include "core/object/ref_counted.h"
#include "core/string/string_name.h"
#include "core/templates/hash_map.h"
#include "core/variant/variant.h"

class ConsoleSignaler : public Object {
	GDCLASS(ConsoleSignaler, Object);
	static void _bind_methods() {
		ADD_SIGNAL(MethodInfo("changed"));
		ADD_SIGNAL(MethodInfo("executed"));
	};
};

class CTokenData {
public:
	enum TokenType {
		VARIABLE,
		COMMAND
	};

private:
	String name;
	TokenType token_type;
	Variant::Type type;
	Variant value;
	ConsoleSignaler *signaler = nullptr;

public:
	CTokenData(){};
	CTokenData(const String &p_name, const Variant::Type &p_type, const Variant &p_default);
	CTokenData(const String &p_name);

	ConsoleSignaler *get_signaler();
	String get_name() const;
	Variant get_value() const;
	void set_value(Variant p_value);
	TokenType get_token_type() const;
	~CTokenData();
};

class ConsoleSystem {
	static constexpr int MAX_CVARS = 64;
	LocalVector<CTokenData> token_datas;
	HashMap<StringName, int> token_map;
	int token_count = 0;

public:
	struct CTextMatch {
		CTokenData *data;
		float similarity;
	};
	CTokenData *create_cvar(const String &p_name, const Variant::Type &p_type, const Variant &p_default);
	CTokenData *create_command(const String &p_name);

	void set(const StringName &p_name, const Variant &p_value);
	CTokenData *get_token(const StringName &p_name);
	int token_match(const String &p_query, CTextMatch *r_out, const int p_count_max);

	void initialize_console();
	void deinit_console();

	static ConsoleSystem *get_singleton();
	void parse_user_command(const String &p_text);

	ConsoleSystem();
	~ConsoleSystem();
};

struct CVar {
	String name;
	CTokenData *data;

	CVar(const char *p_name, const Variant::Type &p_type, const Variant &p_default) {
		data = ConsoleSystem::get_singleton()->create_cvar(p_name, p_type, p_default);
		name = p_name;
	}

	Variant get() const {
		DEV_ASSERT(data != nullptr);
		return data->get_value();
	}
};

struct CCommand {
	String name;
	CTokenData *data;

	CCommand(const char *p_name) {
		data = ConsoleSystem::get_singleton()->create_command(p_name);
		name = p_name;
	}
};

#endif // CVAR_H
