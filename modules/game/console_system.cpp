#include "console_system.h"

#include "core/io/file_access.h"
#include "core/io/dir_access.h"
#include "core/variant/variant_utility.h"

CTokenData *ConsoleSystem::create_cvar(const String &p_name, const Variant::Type &p_type, const Variant &p_default) {
    token_datas[token_count] = CTokenData(p_name, p_type, p_default);
    
    ERR_FAIL_COND_V(token_count >= MAX_CVARS, nullptr);

    token_count++;

    return &token_datas[token_count-1];
}

void ConsoleSystem::set(const StringName &p_name, const Variant &p_value) {
    ERR_FAIL_COND_MSG(!token_map.has(p_name), "CVar doesn't exist");
    token_datas[token_map[p_name]].set_value(p_value);
}

CTokenData *ConsoleSystem::get_token(const StringName &p_name) {
    ERR_FAIL_COND_V(!token_map.has(p_name), nullptr);
    return &token_datas[token_map[p_name]];
}

CTokenData *ConsoleSystem::create_command(const String &p_name) {
    token_datas[token_count] = CTokenData(p_name);
    
    ERR_FAIL_COND_V(token_count >= MAX_CVARS, nullptr);

    token_count++;

    return &token_datas[token_count-1];
}

int ConsoleSystem::token_match(const String &p_query, CTextMatch *r_out, const int p_count_max) {
    int matched_count = 0;
    for (int i = 0; i < token_count; i++) {
        float similarity = token_datas[i].get_name().similarity(p_query);
        if (similarity > 0.0) {
            r_out[matched_count].data = &token_datas[i];
            r_out[matched_count].similarity = similarity;
            matched_count++;
            if (matched_count >= p_count_max) {
                break;
            }
        }
    }
    return matched_count;
}

void ConsoleSystem::deinit_console() {
    token_datas.clear();
}

ConsoleSystem::~ConsoleSystem() {
}

void ConsoleSystem::initialize_console() {
    for (int i = 0; i < token_count; i++) {
        token_map.insert(StringName(token_datas[i].get_name()), i);
    }

    Ref<DirAccess> root = DirAccess::create_for_path("res://");

    if (root->file_exists("autoexec.cfg")) {
        Ref<FileAccess> f = FileAccess::open("res://autoexec.cfg", FileAccess::READ);
        while (!f->eof_reached()) {
            String line = f->get_line();
            parse_user_command(line);
        }
    }

}

ConsoleSystem *ConsoleSystem::get_singleton() {
    static ConsoleSystem singleton {};
    return &singleton;
}

void ConsoleSystem::parse_user_command(const String &p_text) {
    int cvar_space = p_text.find_char(' ');
    
    String token_name = p_text.substr(0, cvar_space).strip_edges();
    String val_string;
    if (cvar_space != -1) {
        val_string = p_text.substr(cvar_space, -1).strip_edges();
    }

    CTokenData *token_data = ConsoleSystem::get_singleton()->get_token(token_name);
    if (!token_data) {
        print_error("CVar or command \"" + token_name + "\" does not exist");
        return;
    }
    
    if (token_data->get_token_type() == CTokenData::VARIABLE) {
        if (!val_string.is_empty()) {
            Variant variant = VariantUtilityFunctions::str_to_var(val_string);

            if (!variant.can_convert(variant.get_type(), token_data->get_value().get_type())) {
                print_error(vformat("Error setting cvar %s: conversion failed", token_name));
                return;
            }
            token_data->set_value(VariantUtilityFunctions::type_convert(variant, token_data->get_value().get_type()));
        }
        print_line(">", token_name, "=", token_data->get_value());
    } else {
        token_data->get_signaler()->emit_signal("executed");
        print_line(">", token_name);
    }
}

ConsoleSystem::ConsoleSystem() {
    token_datas.resize(MAX_CVARS);
}

CTokenData::CTokenData(const String &p_name, const Variant::Type &p_type, const Variant &p_default) {
    name = p_name;
    type = p_type;
    value = p_default;
    token_type = TokenType::VARIABLE;
}

CTokenData::CTokenData(const String &p_name) {
    name = p_name;
    token_type = TokenType::COMMAND;
}

ConsoleSignaler *CTokenData::get_signaler() {
    if (!signaler) {
        signaler = memnew(ConsoleSignaler);
    }
    return signaler;
}

String CTokenData::get_name() const {
    return name;
}

Variant CTokenData::get_value() const {
    return value;
}

void CTokenData::set_value(Variant p_value) {
    value = p_value;
    get_signaler()->emit_signal("changed");
}

CTokenData::TokenType CTokenData::get_token_type() const {
    return token_type;
}

CTokenData::~CTokenData() {
    memdelete_notnull(signaler);
}
