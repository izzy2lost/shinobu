#include "replay.h"
#include "core/io/stream_peer.h"

void HBReplayWriter::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_song_chart_hash", "song_chart_hash"), &HBReplayWriter::set_song_chart_hash);
	ClassDB::bind_method(D_METHOD("get_song_chart_hash"), &HBReplayWriter::get_song_chart_hash);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "song_chart_hash"), "set_song_chart_hash", "get_song_chart_hash");

	ClassDB::bind_method(D_METHOD("set_song_id", "song_id"), &HBReplayWriter::set_song_id);
	ClassDB::bind_method(D_METHOD("get_song_id"), &HBReplayWriter::get_song_id);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "song_id"), "set_song_id", "get_song_id");

	ClassDB::bind_method(D_METHOD("set_song_difficulty", "song_difficulty"), &HBReplayWriter::set_song_difficulty);
	ClassDB::bind_method(D_METHOD("get_song_difficulty"), &HBReplayWriter::get_song_difficulty);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "song_difficulty"), "set_song_difficulty", "get_song_difficulty");

	ClassDB::bind_method(D_METHOD("write_to_buffer"), &HBReplayWriter::write_to_buffer);
	ClassDB::bind_method(D_METHOD("push_event", "event"), &HBReplayWriter::push_event);
}

int HBReplayWriter::_get_joy_device_idx(StringName p_device_guid, const String &p_name) {
	HBReplay::GamepadDeviceInfo *dev_info = gamepad_device_infos.getptr(p_device_guid);
	if (!dev_info) {
		HBReplay::GamepadDeviceInfo new_dev_info = {
			.device_id = static_cast<uint8_t>(gamepad_device_infos.size()),
			.device_name = p_name,
			.device_sdl_guid = p_device_guid,
		};
		gamepad_device_infos.insert(p_device_guid, new_dev_info);
		return new_dev_info.device_id;
	}
	return dev_info->device_id;
}

void HBReplayWriter::push_event(const Ref<HBReplayEvent> &p_event) {
	ERR_FAIL_COND(!p_event.is_valid());
	// Rate limit events set as extra
	if (p_event->get_is_extra()) {
		if ((p_event->get_game_timestamp() >= last_extra_event_time + MAX_EXTRA_EVENT_RATE_W)) {
			last_extra_event_time = p_event->get_game_timestamp();
		} else {
			return;
		}
	}

	int device_idx = -1;
	if (p_event->is_gamepad_event()) {
		device_idx = _get_joy_device_idx(p_event->get_device_guid(), p_event->get_device_name());
	}

	HBReplay::ReplayEvent event{};
	event.event_type = p_event->get_event_type();
	event.press_actions = p_event->get_press_actions();
	event.release_actions = p_event->get_release_actions();
	event.game_timestamp = p_event->get_game_timestamp();

	switch (p_event->get_event_type()) {
		case HBReplay::GAMEPAD_JOY_SINGLE_AXIS: {
			event.event_data.gamepad.device_id = device_idx;
			event.event_data.gamepad.axis.joy_axis[0] = p_event->get_joystick_axis(0);
			event.event_data.gamepad.axis.joystick_position[0] = p_event->get_joystick_position().x;
		} break;
		case HBReplay::GAMEPAD_JOY: {
			event.event_data.gamepad.device_id = device_idx;
			event.event_data.gamepad.axis.joy_axis[0] = p_event->get_joystick_axis(0);
			event.event_data.gamepad.axis.joy_axis[1] = p_event->get_joystick_axis(1);
			event.event_data.gamepad.axis.joystick_position[0] = p_event->get_joystick_position().x;
			event.event_data.gamepad.axis.joystick_position[1] = p_event->get_joystick_position().y;
		} break;
		case HBReplay::GAMEPAD_BUTTON: {
			event.event_data.gamepad.device_id = device_idx;
			event.event_data.gamepad.button.button_pressed = p_event->get_gamepad_button_pressed();
			event.event_data.gamepad.button.gamepad_button_idx = p_event->get_gamepad_button_index();
		} break;
		case HBReplay::KEYBOARD_KEY: {
			event.event_data.keyboard.key_pressed = p_event->get_keyboard_key_pressed();
			event.event_data.keyboard.key = p_event->get_keyboard_key();
		} break;
		default: {
		} break;
	}

	events.push_back(event);

	return;
}

String HBReplayWriter::get_song_id() const { return song_id; }

void HBReplayWriter::set_song_id(const String &p_song_id) { song_id = p_song_id; }

String HBReplayWriter::get_song_difficulty() const { return song_difficulty; }

void HBReplayWriter::set_song_difficulty(const String &p_song_difficulty) { song_difficulty = p_song_difficulty; }

String HBReplayWriter::get_song_chart_hash() const { return song_chart_hash; }

void HBReplayWriter::set_song_chart_hash(const String &p_song_chart_hash) { song_chart_hash = p_song_chart_hash; }

PackedByteArray HBReplayWriter::write_to_buffer() {
	Ref<StreamPeerBuffer> peer_buffer;
	peer_buffer.instantiate();
	peer_buffer->put_u8(0x50); // P
	peer_buffer->put_u8(0x48); // H
	peer_buffer->put_u8(0x52); // R
	peer_buffer->put_u8(VERSION);
	peer_buffer->put_utf8_string(song_id);
	peer_buffer->put_utf8_string(song_difficulty);
	peer_buffer->put_utf8_string(song_chart_hash);

	peer_buffer->put_u8(gamepad_device_infos.size()); // Gamepad device count

	Vector<HBReplay::GamepadDeviceInfo> infos;
	infos.sort_custom<HBReplay::GamepadDeviceInfoComparator>();

	for (KeyValue<StringName, HBReplay::GamepadDeviceInfo> &info : gamepad_device_infos) {
		infos.push_back(info.value);
	}

	for (HBReplay::GamepadDeviceInfo &info : infos) {
		peer_buffer->put_utf8_string(info.device_name);
		peer_buffer->put_utf8_string(info.device_sdl_guid);
	}

	peer_buffer->put_u32(events.size());

	for (const HBReplay::ReplayEvent &event : events) {
		peer_buffer->put_u8(event.event_type);
		peer_buffer->put_64(event.game_timestamp);
		peer_buffer->put_u8(event.press_actions);
		peer_buffer->put_u8(event.release_actions);

		switch (event.event_type) {
			case HBReplay::GAMEPAD_JOY_SINGLE_AXIS: {
				peer_buffer->put_u8(event.event_data.gamepad.device_id);
				peer_buffer->put_u8(event.event_data.gamepad.axis.joy_axis[0]);
				peer_buffer->put_float(event.event_data.gamepad.axis.joystick_position[0]);
			} break;
			case HBReplay::GAMEPAD_JOY: {
				peer_buffer->put_u8(event.event_data.gamepad.device_id);
				peer_buffer->put_u8(event.event_data.gamepad.axis.joy_axis[0]);
				peer_buffer->put_u8(event.event_data.gamepad.axis.joy_axis[1]);
				peer_buffer->put_float(event.event_data.gamepad.axis.joystick_position[0]);
				peer_buffer->put_float(event.event_data.gamepad.axis.joystick_position[1]);
			} break;
			case HBReplay::GAMEPAD_BUTTON: {
				peer_buffer->put_u8(event.event_data.gamepad.device_id);
				peer_buffer->put_u8(event.event_data.gamepad.button.button_pressed);
				peer_buffer->put_u8(event.event_data.gamepad.button.gamepad_button_idx);
			} break;
			case HBReplay::KEYBOARD_KEY: {
				peer_buffer->put_u8(event.event_data.keyboard.key_pressed);
				peer_buffer->put_u32(event.event_data.keyboard.key);
			} break;
			default: {
			}
		}
	}
	return peer_buffer->get_data_array();
}

void HBReplayEvent::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_event_type"), &HBReplayEvent::get_event_type);
	ClassDB::bind_method(D_METHOD("set_event_type", "event_type"), &HBReplayEvent::set_event_type);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "event_type"), "set_event_type", "get_event_type");

	ClassDB::bind_method(D_METHOD("get_game_timestamp"), &HBReplayEvent::get_game_timestamp);
	ClassDB::bind_method(D_METHOD("set_game_timestamp", "game_timestamp"), &HBReplayEvent::set_game_timestamp);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "game_timestamp"), "set_game_timestamp", "get_game_timestamp");

	ClassDB::bind_method(D_METHOD("get_press_actions"), &HBReplayEvent::get_press_actions);
	ClassDB::bind_method(D_METHOD("set_press_actions", "press_actions"), &HBReplayEvent::set_press_actions);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "press_actions"), "set_press_actions", "get_press_actions");

	ClassDB::bind_method(D_METHOD("get_release_actions"), &HBReplayEvent::get_release_actions);
	ClassDB::bind_method(D_METHOD("set_release_actions", "release_actions"), &HBReplayEvent::set_release_actions);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "release_actions"), "set_release_actions", "get_release_actions");

	ClassDB::bind_method(D_METHOD("get_joystick_position"), &HBReplayEvent::get_joystick_position);
	ClassDB::bind_method(D_METHOD("set_joystick_position", "joystick_position"), &HBReplayEvent::set_joystick_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "joystick_position"), "set_joystick_position", "get_joystick_position");

	ClassDB::bind_method(D_METHOD("get_joystick_axis", "index"), &HBReplayEvent::get_joystick_axis);
	ClassDB::bind_method(D_METHOD("set_joystick_axis", "index", "axis"), &HBReplayEvent::set_joystick_axis);

	ClassDB::bind_method(D_METHOD("get_gamepad_button_index"), &HBReplayEvent::get_gamepad_button_index);
	ClassDB::bind_method(D_METHOD("set_gamepad_button_index", "gamepad_button_index"), &HBReplayEvent::set_gamepad_button_index);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "gamepad_button_index"), "set_gamepad_button_index", "get_gamepad_button_index");

	ClassDB::bind_method(D_METHOD("get_gamepad_button_pressed"), &HBReplayEvent::get_gamepad_button_pressed);
	ClassDB::bind_method(D_METHOD("set_gamepad_button_pressed", "gamepad_button_pressed"), &HBReplayEvent::set_gamepad_button_pressed);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "gamepad_button_pressed"), "set_gamepad_button_pressed", "get_gamepad_button_pressed");

	ClassDB::bind_method(D_METHOD("get_keyboard_key"), &HBReplayEvent::get_keyboard_key);
	ClassDB::bind_method(D_METHOD("set_keyboard_key", "keyboard_key"), &HBReplayEvent::set_keyboard_key);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "keyboard_key"), "set_keyboard_key", "get_keyboard_key");

	ClassDB::bind_method(D_METHOD("get_keyboard_key_pressed"), &HBReplayEvent::get_keyboard_key_pressed);
	ClassDB::bind_method(D_METHOD("set_keyboard_key_pressed", "keyboard_key_pressed"), &HBReplayEvent::set_keyboard_key_pressed);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "keyboard_key_pressed"), "set_keyboard_key_pressed", "get_keyboard_key_pressed");

	ClassDB::bind_method(D_METHOD("get_is_extra"), &HBReplayEvent::get_is_extra);
	ClassDB::bind_method(D_METHOD("set_is_extra", "is_extra"), &HBReplayEvent::set_is_extra);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_extra"), "set_is_extra", "get_is_extra");

	ClassDB::bind_method(D_METHOD("get_device_guid"), &HBReplayEvent::get_device_guid);
	ClassDB::bind_method(D_METHOD("set_device_guid", "device_guid"), &HBReplayEvent::set_device_guid);
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "device_guid"), "set_device_guid", "get_device_guid");

	ClassDB::bind_method(D_METHOD("get_device_name"), &HBReplayEvent::get_device_name);
	ClassDB::bind_method(D_METHOD("set_device_name", "device_name"), &HBReplayEvent::set_device_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "device_name"), "set_device_name", "get_device_name");

	ClassDB::bind_method(D_METHOD("get_state_snapshot"), &HBReplayEvent::get_state_snapshot_bind);
}

HBReplay::EventType HBReplayEvent::get_event_type() const { return event_type; }

void HBReplayEvent::set_event_type(HBReplay::EventType p_event_type) {
	event_type = p_event_type;
	switch (event_type) {
		case HBReplay::GAMEPAD_JOY_SINGLE_AXIS:
		case HBReplay::GAMEPAD_JOY: {
			event_data.gamepad.device_id = 0;
			event_data.gamepad.axis.joy_axis[0] = (uint8_t)JoyAxis::INVALID;
			event_data.gamepad.axis.joy_axis[1] = (uint8_t)JoyAxis::INVALID;
			event_data.gamepad.axis.joystick_position[0] = 0.0;
			event_data.gamepad.axis.joystick_position[1] = 0.0;
		} break;
		case HBReplay::GAMEPAD_BUTTON: {
			event_data.gamepad.device_id = 0;
			event_data.gamepad.button.button_pressed = false;
			event_data.gamepad.button.gamepad_button_idx = (uint8_t)JoyButton::INVALID;
		} break;
		case HBReplay::KEYBOARD_KEY: {
			event_data.keyboard.key_pressed = false;
			event_data.keyboard.key = (uint64_t)Key::A;
		} break;
		case HBReplay::EVENT_MAX: {
			DEV_ASSERT(false);
		} break;
	}
}

int64_t HBReplayEvent::get_game_timestamp() const { return game_timestamp; }

void HBReplayEvent::set_game_timestamp(const int64_t &p_game_timestamp) { game_timestamp = p_game_timestamp; }

BitField<HBReplay::EventActionBitfield> HBReplayEvent::get_release_actions() const {
	return release_actions;
}

void HBReplayEvent::set_release_actions(BitField<HBReplay::EventActionBitfield> p_release_actions) {
	release_actions = p_release_actions;
}

BitField<HBReplay::EventActionBitfield> HBReplayEvent::get_press_actions() const {
	return press_actions;
}

void HBReplayEvent::set_press_actions(BitField<HBReplay::EventActionBitfield> p_press_actions) {
	press_actions = p_press_actions;
}

void HBReplayEvent::set_joystick_position(Vector2 p_joystick_position) {
	ERR_FAIL_COND(event_type != HBReplay::GAMEPAD_JOY && event_type != HBReplay::GAMEPAD_JOY_SINGLE_AXIS);
	event_data.gamepad.axis.joystick_position[0] = p_joystick_position.x;
	event_data.gamepad.axis.joystick_position[1] = p_joystick_position.y;
}

Vector2 HBReplayEvent::get_joystick_position() const {
	ERR_FAIL_COND_V(event_type != HBReplay::GAMEPAD_JOY && event_type != HBReplay::GAMEPAD_JOY_SINGLE_AXIS, Vector2());
	return Vector2(event_data.gamepad.axis.joystick_position[0], event_data.gamepad.axis.joystick_position[1]);
}

void HBReplayEvent::set_joystick_axis(int p_idx, JoyAxis p_axis) {
	ERR_FAIL_COND(event_type != HBReplay::GAMEPAD_JOY && event_type != HBReplay::GAMEPAD_JOY_SINGLE_AXIS);
	ERR_FAIL_COND(p_idx > 1);
	ERR_FAIL_COND(p_idx == 1 && event_type != HBReplay::GAMEPAD_JOY);

	event_data.gamepad.axis.joy_axis[p_idx] = static_cast<uint8_t>(p_axis);
}

uint8_t HBReplayEvent::get_joystick_axis(int p_idx) {
	ERR_FAIL_COND_V(event_type != HBReplay::GAMEPAD_JOY && event_type != HBReplay::GAMEPAD_JOY_SINGLE_AXIS, 0);
	ERR_FAIL_COND_V(p_idx > 1, 0);
	ERR_FAIL_COND_V(p_idx == 1 && event_type != HBReplay::GAMEPAD_JOY, 0);

	return event_data.gamepad.axis.joy_axis[p_idx];
}

void HBReplayEvent::set_gamepad_button_pressed(bool p_gamepad_button_pressed) {
	ERR_FAIL_COND(event_type != HBReplay::GAMEPAD_BUTTON);
	event_data.gamepad.button.button_pressed = p_gamepad_button_pressed;
}

bool HBReplayEvent::get_gamepad_button_pressed() const {
	ERR_FAIL_COND_V(event_type != HBReplay::GAMEPAD_BUTTON, false);
	return event_data.gamepad.button.button_pressed;
}

void HBReplayEvent::set_gamepad_button_index(uint8_t p_button_index) {
	ERR_FAIL_COND(event_type != HBReplay::GAMEPAD_BUTTON);
	event_data.gamepad.button.gamepad_button_idx = p_button_index;
}

uint8_t HBReplayEvent::get_gamepad_button_index() {
	ERR_FAIL_COND_V(event_type != HBReplay::GAMEPAD_BUTTON, 0);
	return event_data.gamepad.button.gamepad_button_idx;
}

void HBReplayEvent::set_keyboard_key_pressed(bool p_keyboard_key_pressed) {
	ERR_FAIL_COND(event_type != HBReplay::KEYBOARD_KEY);
	event_data.keyboard.key_pressed = p_keyboard_key_pressed;
};

bool HBReplayEvent::get_keyboard_key_pressed() const {
	ERR_FAIL_COND_V(event_type != HBReplay::KEYBOARD_KEY, false);
	return event_data.keyboard.key_pressed;
};

void HBReplayEvent::set_keyboard_key(uint64_t p_button_index) {
	ERR_FAIL_COND(event_type != HBReplay::KEYBOARD_KEY);
	event_data.keyboard.key = p_button_index;
}

uint64_t HBReplayEvent::get_keyboard_key() {
	ERR_FAIL_COND_V(event_type != HBReplay::KEYBOARD_KEY, 0);
	return event_data.keyboard.key;
}

void HBReplayEvent::set_is_extra(bool p_is_extra) {
	is_extra = p_is_extra;
}

bool HBReplayEvent::get_is_extra() const {
	return is_extra;
}

void HBReplayEvent::set_device_name(String p_device_name) { device_name = p_device_name; }

void HBReplayEvent::set_device_guid(StringName p_device_guid) { device_guid = p_device_guid; }

String HBReplayEvent::get_device_name() const { return device_name; }

StringName HBReplayEvent::get_device_guid() const { return device_guid; }

void HBReplayEvent::get_state_snapshot(HBReplay::StateSnapshot *p_state_snapshot) const { *p_state_snapshot = state_snapshot; }

void HBReplayEvent::set_state_snapshot(const HBReplay::StateSnapshot &p_state_snapshot) { state_snapshot = p_state_snapshot; }

void HBReplayReader::_show_error(Error p_error, String message) {
	error = p_error;
	print_error("Error parsing replay file: " + message);
}

bool HBReplayReader::_read_event(const Ref<StreamPeerBuffer> p_spb) {
	uint8_t event_type = p_spb->get_u8();
	if (event_type >= HBReplay::EVENT_MAX) {
		_show_error(ERR_PARSE_ERROR, "Event type was invalid");
		return false;
	}

	HBReplay::ReplayEvent event{
		.event_type = (HBReplay::EventType)event_type,
		.game_timestamp = p_spb->get_64(),
		.press_actions = p_spb->get_u8(),
		.release_actions = p_spb->get_u8(),
	};

	switch ((HBReplay::EventType)event_type) {
		case HBReplay::GAMEPAD_JOY_SINGLE_AXIS: {
			event.event_data.gamepad.device_id = p_spb->get_u8();
			event.event_data.gamepad.axis.joy_axis[0] = p_spb->get_u8();
			event.event_data.gamepad.axis.joystick_position[0] = p_spb->get_float();
			if (event.event_data.gamepad.axis.joy_axis[0] >= (int)JoyAxis::MAX) {
				_show_error(ERR_PARSE_ERROR, "Invalid joypad axis");
			}
		} break;
		case HBReplay::GAMEPAD_JOY: {
			event.event_data.gamepad.device_id = p_spb->get_u8();
			event.event_data.gamepad.axis.joy_axis[0] = p_spb->get_u8();
			event.event_data.gamepad.axis.joy_axis[1] = p_spb->get_u8();
			event.event_data.gamepad.axis.joystick_position[0] = p_spb->get_float();
			event.event_data.gamepad.axis.joystick_position[1] = p_spb->get_float();

			if (event.event_data.gamepad.axis.joy_axis[0] >= (int)JoyAxis::MAX) {
				_show_error(ERR_PARSE_ERROR, "Invalid joypad axis 0");
			}
			if (event.event_data.gamepad.axis.joy_axis[1] >= (int)JoyAxis::MAX) {
				_show_error(ERR_PARSE_ERROR, "Invalid joypad axis 1");
			}
		} break;
		case HBReplay::GAMEPAD_BUTTON: {
			event.event_data.gamepad.device_id = p_spb->get_u8();
			event.event_data.gamepad.button.button_pressed = p_spb->get_u8();
			event.event_data.gamepad.button.gamepad_button_idx = p_spb->get_u8();
			if (event.event_data.gamepad.button.gamepad_button_idx >= (int)JoyButton::MAX) {
				_show_error(ERR_PARSE_ERROR, "Invalid joypad button");
			}
		} break;
		case HBReplay::KEYBOARD_KEY: {
			event.event_data.keyboard.key_pressed = p_spb->get_u8();
			event.event_data.keyboard.key = p_spb->get_u32();
		} break;
		case HBReplay::EVENT_MAX: {
		} break;
	}

	replay_events.push_back(event);

	return true;
}

void HBReplayReader::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_replay_events_in_interval", "start", "end"), &HBReplayReader::get_replay_events_in_interval_bind);
	ClassDB::bind_static_method(SNAME("HBReplayReader"), D_METHOD("from_buffer", "buffer"), &HBReplayReader::from_buffer);
	ClassDB::bind_method(D_METHOD("get_gamepad_info_count"), &HBReplayReader::get_gamepad_info_count);
	ClassDB::bind_method(D_METHOD("get_gamepad_guid"), &HBReplayReader::get_gamepad_guid);
}

Ref<HBReplayReader> HBReplayReader::from_buffer(const PackedByteArray &p_buffer) {
	Ref<HBReplayReader> reader;
	reader.instantiate();

	Ref<StreamPeerBuffer> spb;
	spb.instantiate();
	spb->set_data_array(p_buffer);

	struct {
		uint8_t header[3];
		uint8_t version = 0;
		String song_id;
		String song_difficulty;
		String song_chart_hash;
		uint8_t gamepad_device_info_count;
	} replay_header;

	spb->get_data(replay_header.header, 3);

	if (!(replay_header.header[0] == 'P' && replay_header.header[1] == 'H' && replay_header.header[2] == 'R')) {
		reader->_show_error(ERR_PARSE_ERROR, "Header didnt match");
		return reader;
	}
	replay_header.version = spb->get_u8();
	replay_header.song_id = spb->get_utf8_string();
	replay_header.song_difficulty = spb->get_utf8_string();
	replay_header.song_chart_hash = spb->get_utf8_string();
	replay_header.gamepad_device_info_count = spb->get_u8();

	reader->song_id = replay_header.song_id;
	reader->song_difficulty = replay_header.song_difficulty;
	reader->song_chart_hash = replay_header.song_chart_hash;

	for (uint8_t i = 0; i < replay_header.gamepad_device_info_count; i++) {
		HBReplay::GamepadDeviceInfo device_info{
			.device_id = i,
			.device_name = spb->get_utf8_string(),
			.device_sdl_guid = spb->get_utf8_string()
		};

		reader->gamepad_device_infos.push_back(device_info);
	}

	uint32_t event_count = spb->get_u32();

	for (uint32_t i = 0; i < event_count; i++) {
		if (!reader->_read_event(spb)) {
			return reader;
		}
	}

	// Now time to generate state snapshots
	HBReplay::StateSnapshot current_snapshot = HBReplay::StateSnapshot();

	for (int i = 0; i < reader->gamepad_device_infos.size(); i++) {
		current_snapshot.joypad_state.insert(reader->gamepad_device_infos[i].device_sdl_guid, HBReplay::JoypadState());
	}

	reader->state_snapshots.resize(reader->replay_events.size() + 1);

	HBReplay::StateSnapshot *sn = reader->state_snapshots.ptrw();
	sn[0] = current_snapshot;

	for (int i = 0; i < reader->replay_events.size(); i++) {
		const HBReplay::ReplayEvent &event = reader->replay_events[i];

		bool event_pressed = event.press_actions != 0;

		HBReplay::EventActionBitfield press_actions = (HBReplay::EventActionBitfield)(uint8_t)event.press_actions;
		HBReplay::EventActionBitfield release_actions = (HBReplay::EventActionBitfield)(uint8_t)event.release_actions;

		switch (event.event_type) {
			case HBReplay::EventType::GAMEPAD_BUTTON: {
				int gp_device = event.event_data.gamepad.device_id;
				int gp_button = event.event_data.gamepad.button.gamepad_button_idx;
				GamepadGUID gp_device_guid = reader->gamepad_device_infos[gp_device].device_sdl_guid;
				current_snapshot.joypad_state[gp_device_guid].button_state[gp_button] = event_pressed;
				current_snapshot.joypad_state[gp_device_guid].button_action_state[gp_button].set_flag(press_actions);
				current_snapshot.joypad_state[gp_device_guid].button_action_state[gp_button].clear_flag(release_actions);
				current_snapshot.joypad_state[gp_device_guid].button_state[gp_button] = event.event_data.gamepad.button.button_pressed;
			} break;
			case HBReplay::GAMEPAD_JOY_SINGLE_AXIS: {
				int gp_device = event.event_data.gamepad.device_id;
				int gp_axis = event.event_data.gamepad.axis.joy_axis[0];
				GamepadGUID gp_device_guid = reader->gamepad_device_infos[gp_device].device_sdl_guid;

				current_snapshot.joypad_state[gp_device_guid].axis_state[gp_axis] = event.event_data.gamepad.axis.joystick_position[0];

				current_snapshot.joypad_state[gp_device_guid].axis_action_state[gp_axis].set_flag(press_actions);
				current_snapshot.joypad_state[gp_device_guid].axis_action_state[gp_axis].clear_flag(release_actions);
			} break;
			case HBReplay::GAMEPAD_JOY: {
				int gp_device = event.event_data.gamepad.device_id;

				for (int gp_axis_i = 0; gp_axis_i < 2; gp_axis_i++) {
					int gp_axis = event.event_data.gamepad.axis.joy_axis[gp_axis_i];
					GamepadGUID gp_device_guid = reader->gamepad_device_infos[gp_device].device_sdl_guid;

					current_snapshot.joypad_state[gp_device_guid].axis_state[gp_axis] = event.event_data.gamepad.axis.joystick_position[gp_axis_i];
					if (gp_axis_i == 0) {
						current_snapshot.joypad_state[gp_device_guid].axis_action_state[gp_axis].set_flag(press_actions);
						current_snapshot.joypad_state[gp_device_guid].axis_action_state[gp_axis].clear_flag(release_actions);
					}
				}
			} break;
			case HBReplay::KEYBOARD_KEY: {
				Key k = (Key)event.event_data.keyboard.key;
				bool key_pressed = event.event_data.keyboard.key_pressed;
				if (!key_pressed && current_snapshot.pressed_keys.has(k)) {
					current_snapshot.pressed_keys.erase(k);
				} else if (key_pressed && !current_snapshot.pressed_keys.has(k)) {
					current_snapshot.pressed_keys.insert(k);
				}

				if (current_snapshot.pressed_key_action_state.has(k)) {
					current_snapshot.pressed_key_action_state.erase(k);
				}

				BitField<HBReplay::EventActionBitfield> value;
				value.set_flag(press_actions);
				current_snapshot.pressed_key_action_state.insert(k, press_actions);

			} break;
			case HBReplay::EVENT_MAX: {
				DEV_ASSERT(false);

			} break;
		}

		// Recalculate action held count vector

		for (int j = 0; j < HBReplay::EventAction::NOTE_MAX; j++) {
			current_snapshot.action_held_count[j] = 0;
			HBReplay::EventActionBitfield action = (HBReplay::EventActionBitfield)(1 << (j));
			for (KeyValue<Key, BitField<HBReplay::EventActionBitfield>> &kv : current_snapshot.pressed_key_action_state) {
				if (kv.value.has_flag(action)) {
					current_snapshot.action_held_count[j] += 1;
				}
			}

			for (KeyValue<StringName, HBReplay::JoypadState> &kv : current_snapshot.joypad_state) {
				for (size_t k = 0; k < std::size(kv.value.axis_action_state); k++) {
					if (kv.value.axis_action_state[k].has_flag(action)) {
						current_snapshot.action_held_count[j] += 1;
					}
				}

				for (size_t k = 0; k < std::size(kv.value.button_action_state); k++) {
					if (kv.value.button_action_state[k].has_flag(action)) {
						current_snapshot.action_held_count[j] += 1;
					}
				}
			}
		}

		sn[i + 1] = current_snapshot;
	}
	return reader;
}

int HBReplayReader::get_gamepad_info_count() const {
	return gamepad_device_infos.size();
}

String HBReplayReader::get_gamepad_name(int p_idx) const {
	ERR_FAIL_INDEX_V(p_idx, gamepad_device_infos.size(), "");
	return gamepad_device_infos[p_idx].device_name;
}

StringName HBReplayReader::get_gamepad_guid(int p_idx) const {
	ERR_FAIL_INDEX_V(p_idx, gamepad_device_infos.size(), "");
	return gamepad_device_infos[p_idx].device_sdl_guid;
}

Vector<Ref<HBReplayEvent>> HBReplayReader::get_replay_events_in_interval(int64_t p_start, int64_t p_end) {
	Vector<Ref<HBReplayEvent>> events_out;
	ERR_FAIL_COND_V(p_start > p_end, events_out);

	// Dummy
	HBReplay::ReplayEvent dummy{};
	dummy.game_timestamp = p_start;
	int start = replay_events.bsearch_custom<ReplayEventComparator>(dummy, true);

	if (start >= replay_events.size() || start < 0 || replay_events.size() == 0) {
		return events_out;
	}

	for (int i = start; i < replay_events.size(); i++) {
		if (replay_events[i].game_timestamp < p_start) {
			continue;
		}

		if (replay_events[i].game_timestamp >= p_end) {
			break;
		}

		Ref<HBReplayEvent> ev;
		ev.instantiate();

		ev->set_event_type(replay_events[i].event_type);
		ev->set_game_timestamp(replay_events[i].game_timestamp);
		ev->set_press_actions(replay_events[i].press_actions);
		ev->set_release_actions(replay_events[i].release_actions);
		ev->set_release_actions(replay_events[i].release_actions);
		const HBReplay::StateSnapshot &snp = state_snapshots[i + 1];

		ev->set_state_snapshot(snp);

		if (ev->is_gamepad_event()) {
			uint8_t device_id = replay_events[i].event_data.gamepad.device_id;
			ev->set_device_guid(get_gamepad_guid(device_id));
			ev->set_device_name(get_gamepad_name(device_id));
		}

		switch (replay_events[i].event_type) {
			case HBReplay::GAMEPAD_JOY_SINGLE_AXIS: {
				ev->set_joystick_axis(0, (JoyAxis)replay_events[i].event_data.gamepad.axis.joy_axis[0]);
				ev->set_joystick_position(Vector2(replay_events[i].event_data.gamepad.axis.joystick_position[0], 0.0));
			} break;
			case HBReplay::GAMEPAD_JOY: {
				ev->set_joystick_axis(0, (JoyAxis)replay_events[i].event_data.gamepad.axis.joy_axis[0]);
				ev->set_joystick_axis(1, (JoyAxis)replay_events[i].event_data.gamepad.axis.joy_axis[1]);
				ev->set_joystick_position(Vector2(replay_events[i].event_data.gamepad.axis.joystick_position[0], replay_events[i].event_data.gamepad.axis.joystick_position[1]));
			} break;
			case HBReplay::GAMEPAD_BUTTON: {
				ev->set_gamepad_button_pressed(replay_events[i].event_data.gamepad.button.button_pressed);
				ev->set_gamepad_button_index(replay_events[i].event_data.gamepad.button.gamepad_button_idx);
			} break;
			case HBReplay::KEYBOARD_KEY: {
				ev->set_keyboard_key_pressed(replay_events[i].event_data.keyboard.key_pressed);
				ev->set_keyboard_key(replay_events[i].event_data.keyboard.key);
			} break;
			case HBReplay::EVENT_MAX:
				break;
		};
		ev->set_press_actions(replay_events[i].press_actions);
		ev->set_release_actions(replay_events[i].release_actions);
		events_out.push_back(ev);
	}

	return events_out;
}

TypedArray<HBReplayEvent> HBReplayReader::get_replay_events_in_interval_bind(int64_t p_start, int64_t p_end) {
	TypedArray<HBReplayEvent> out;

	for (Ref<HBReplayEvent> ev : get_replay_events_in_interval(p_start, p_end)) {
		out.push_back(ev);
	}

	return out;
}

bool HBReplayReader::ReplayEventComparator::operator()(const HBReplay::ReplayEvent &p_left, const HBReplay::ReplayEvent &p_right) const {
	return p_left.game_timestamp < p_right.game_timestamp;
}

void HBReplay::_bind_methods() {
	BIND_BITFIELD_FLAG(NOTE_UP_B);
	BIND_BITFIELD_FLAG(NOTE_LEFT_B);
	BIND_BITFIELD_FLAG(NOTE_DOWN_B);
	BIND_BITFIELD_FLAG(NOTE_RIGHT_B);
	BIND_BITFIELD_FLAG(SLIDE_LEFT_B);
	BIND_BITFIELD_FLAG(SLIDE_RIGHT_B);
	BIND_BITFIELD_FLAG(HEART_NOTE_B);

	BIND_ENUM_CONSTANT(NOTE_UP);
	BIND_ENUM_CONSTANT(NOTE_LEFT);
	BIND_ENUM_CONSTANT(NOTE_DOWN);
	BIND_ENUM_CONSTANT(NOTE_RIGHT);
	BIND_ENUM_CONSTANT(SLIDE_LEFT);
	BIND_ENUM_CONSTANT(SLIDE_RIGHT);
	BIND_ENUM_CONSTANT(HEART_NOTE);
	BIND_ENUM_CONSTANT(NOTE_MAX);

	BIND_ENUM_CONSTANT(GAMEPAD_JOY_SINGLE_AXIS);
	BIND_ENUM_CONSTANT(GAMEPAD_JOY);
	BIND_ENUM_CONSTANT(GAMEPAD_BUTTON);
	BIND_ENUM_CONSTANT(KEYBOARD_KEY);
}

void HBReplayStateSnapshot::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_action_held_count", "action"), &HBReplayStateSnapshot::get_action_held_count);
}
