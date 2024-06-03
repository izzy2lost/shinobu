
#include "core/io/stream_peer.h"
#include "core/object/ref_counted.h"
#include "core/string/ustring.h"
#include "core/variant/type_info.h"
#include "ph_singleton.h"

#include <cstdint>

class HBReplayEvent;

class HBReplay : public RefCounted {
	GDCLASS(HBReplay, RefCounted);

public:
	enum EventType : uint8_t {
		GAMEPAD_JOY_SINGLE_AXIS,
		GAMEPAD_JOY,
		GAMEPAD_BUTTON,
		KEYBOARD_KEY,
		EVENT_MAX
	};

	enum EventActionBitfield : uint8_t {
		NOTE_UP_B = 1,
		NOTE_LEFT_B = 1 << 1,
		NOTE_DOWN_B = 1 << 2,
		NOTE_RIGHT_B = 1 << 3,
		SLIDE_LEFT_B = 1 << 4,
		SLIDE_RIGHT_B = 1 << 5,
		HEART_NOTE_B = 1 << 6
	};

	enum EventAction : uint8_t {
		NOTE_UP,
		NOTE_LEFT,
		NOTE_DOWN,
		NOTE_RIGHT,
		SLIDE_LEFT,
		SLIDE_RIGHT,
		HEART_NOTE,
		NOTE_MAX
	};

	struct JoypadState {
		std::array<float, (int)JoyAxis::MAX> axis_state = {};
		std::array<bool, (int)JoyButton::MAX> button_state = {};
		std::array<BitField<EventActionBitfield>, (int)JoyAxis::MAX> axis_action_state = {};
		std::array<BitField<EventActionBitfield>, (int)JoyButton::MAX> button_action_state = {};
	};

	struct StateSnapshot {
		HashMap<StringName, JoypadState> joypad_state;
		RBSet<Key> pressed_keys;
		RBMap<Key, BitField<EventActionBitfield>> pressed_key_action_state;

		// actually cached
		std::array<uint32_t, (int)EventAction::NOTE_MAX> action_held_count = {};
	};

	struct EventData {
		struct {
			uint8_t device_id;
			struct {
				uint8_t joy_axis[2] = { (uint8_t)JoyAxis::INVALID, (uint8_t)JoyAxis::INVALID };
				float joystick_position[2] = { 0.0, 0.0 };
			} axis;
			struct {
				bool button_pressed;
				uint8_t gamepad_button_idx;
			} button;
		} gamepad;

		struct {
			bool key_pressed;
			uint32_t key;
		} keyboard;
	};

	struct ReplayEvent {
		EventType event_type;
		int64_t game_timestamp;
		BitField<EventActionBitfield> press_actions;
		BitField<EventActionBitfield> release_actions;
		EventData event_data;
	};

	struct GamepadDeviceInfo {
		uint8_t device_id;
		String device_name;
		StringName device_sdl_guid;
	};

	struct GamepadDeviceInfoComparator {
		bool operator()(const GamepadDeviceInfo *p_left, const GamepadDeviceInfo *p_right) const {
			return p_left->device_id < p_right->device_id;
		}
		bool operator()(const GamepadDeviceInfo &p_left, const GamepadDeviceInfo &p_right) const {
			return p_left.device_id < p_right.device_id;
		}
	};

	typedef StringName GamepadGUID;

protected:
	static void _bind_methods();
};

class HBReplayStateSnapshot : public RefCounted {
	GDCLASS(HBReplayStateSnapshot, RefCounted);
	HBReplay::StateSnapshot state_snapshot;

protected:
	static void _bind_methods();

public:
	uint32_t get_action_held_count(HBReplay::EventAction p_action) const {
		ERR_FAIL_COND_V(p_action >= HBReplay::EventAction::NOTE_MAX, 0);
		PackedInt32Array out;

		return state_snapshot.action_held_count[p_action];
	}

	HBReplayStateSnapshot(HBReplay::StateSnapshot &p_state_snapshot) {
		state_snapshot = p_state_snapshot;
	}
};

class HBReplayWriter : public RefCounted {
	GDCLASS(HBReplayWriter, RefCounted);

	String _error;
	int _error_code = OK;

protected:
	static void _bind_methods();

public:
	static constexpr uint8_t VERSION = 0;

	static constexpr int MAX_EXTRA_EVENT_RATE = 60;
	static constexpr int MAX_EXTRA_EVENT_RATE_W = 1000000 / MAX_EXTRA_EVENT_RATE;
	int64_t last_extra_event_time = 0;

private:
	void _show_error(int p_error_code, const String &p_err_message) {
		_error = p_err_message;
		_error_code = p_error_code;
	}

public:
	HBReplay::EventData event_data;

private:
	String song_id;
	String song_difficulty;
	String song_chart_hash;
	HashMap<HBReplay::GamepadGUID, HBReplay::GamepadDeviceInfo> gamepad_device_infos;
	Vector<HBReplay::ReplayEvent> events;

	int _get_joy_device_idx(StringName p_device_guid, const String &p_name);

public:
	void push_event(const Ref<HBReplayEvent> &p_event);
	String get_song_id() const;
	void set_song_id(const String &p_song_id);
	String get_song_difficulty() const;
	void set_song_difficulty(const String &p_song_difficulty);
	String get_song_chart_hash() const;
	void set_song_chart_hash(const String &p_song_chart_hash);
	PackedByteArray write_to_buffer();
};

class HBReplayReader : public RefCounted {
	GDCLASS(HBReplayReader, RefCounted);

	typedef StringName GamepadGUID;

	Error error = OK;

	Vector<HBReplay::GamepadDeviceInfo> gamepad_device_infos;
	Vector<HBReplay::ReplayEvent> replay_events;

	String song_id;
	String song_difficulty;
	String song_chart_hash;

	struct ReplayEventComparator {
		bool operator()(const HBReplay::ReplayEvent &p_left, const HBReplay::ReplayEvent &p_right) const;
	};

private:
	Vector<HBReplay::StateSnapshot> state_snapshots;

	void _show_error(Error p_error, String message);

	bool _read_event(const Ref<StreamPeerBuffer> p_spb);

protected:
	static void _bind_methods();

public:
	static Ref<HBReplayReader> from_buffer(const PackedByteArray &p_buffer);

	int get_gamepad_info_count() const;

	String get_gamepad_name(int p_idx) const;

	StringName get_gamepad_guid(int p_idx) const;

	Vector<Ref<HBReplayEvent>> get_replay_events_in_interval(int64_t p_start, int64_t p_end);
	TypedArray<HBReplayEvent> get_replay_events_in_interval_bind(int64_t p_start, int64_t p_end);
};

class HBReplayEvent : public RefCounted {
	GDCLASS(HBReplayEvent, RefCounted);

private:
	HBReplay::EventType event_type = HBReplay::EventType::GAMEPAD_JOY;
	int64_t game_timestamp;
	BitField<HBReplay::EventActionBitfield> press_actions;
	BitField<HBReplay::EventActionBitfield> release_actions;
	HBReplay::StateSnapshot state_snapshot = {};
	bool is_extra = false;
	String device_name;
	StringName device_guid;

	HBReplay::EventData event_data = {};

protected:
	static void _bind_methods();

public:
	bool is_gamepad_event() const {
		return event_type < HBReplay::KEYBOARD_KEY;
	}

	HBReplay::EventType get_event_type() const;
	void set_event_type(HBReplay::EventType p_event_type);

	int64_t get_game_timestamp() const;
	void set_game_timestamp(const int64_t &p_game_timestamp);

	BitField<HBReplay::EventActionBitfield> get_release_actions() const;
	void set_release_actions(BitField<HBReplay::EventActionBitfield> p_release_actions);

	BitField<HBReplay::EventActionBitfield> get_press_actions() const;
	void set_press_actions(BitField<HBReplay::EventActionBitfield> p_press_actions);

	void set_joystick_position(Vector2 p_joystick_position);

	Vector2 get_joystick_position() const;

	void set_joystick_axis(int p_idx, JoyAxis p_axis);

	uint8_t get_joystick_axis(int p_idx);

	void set_gamepad_button_pressed(bool p_gamepad_button_pressed);
	bool get_gamepad_button_pressed() const;
	void set_gamepad_button_index(uint8_t p_button_index);
	uint8_t get_gamepad_button_index();

	void set_keyboard_key_pressed(bool p_keyboard_key_pressed);
	bool get_keyboard_key_pressed() const;
	void set_keyboard_key(uint64_t p_button_index);
	uint64_t get_keyboard_key();

	void set_is_extra(bool p_is_extra);

	bool get_is_extra() const;

	void set_device_name(String p_device_name);
	void set_device_guid(StringName p_device_guid);

	String get_device_name() const;
	StringName get_device_guid() const;

	void get_state_snapshot(HBReplay::StateSnapshot *p_state_snapshot) const;
	void set_state_snapshot(const HBReplay::StateSnapshot &p_state_snapshot);

	Ref<HBReplayStateSnapshot> get_state_snapshot_bind() const {
		Ref<HBReplayStateSnapshot> snp;
		snp.instantiate(state_snapshot);
		return snp;
	};
};

class HBReplaySnapshot : public RefCounted {
	GDCLASS(HBReplaySnapshot, RefCounted);

public:
};

VARIANT_ENUM_CAST(HBReplay::EventType);
VARIANT_ENUM_CAST(HBReplay::EventAction);
VARIANT_BITFIELD_CAST(HBReplay::EventActionBitfield);