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
