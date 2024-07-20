/**************************************************************************/
/*  test_steamworks.h                                                     */
/**************************************************************************/
/*                         This file is part of:                          */
/*                           EIRTeam.Steamworks                           */
/*                         https://ph.eirteam.moe                         */
/**************************************************************************/
/* Copyright (c) 2023-present Álex Román (EIRTeam) & contributors.        */
/*                                                                        */
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

#ifndef TESTS_HB_REPLAY_H
#define TESTS_HB_REPLAY_H

#include "../replay.h"
#include "tests/test_macros.h"
#include "tests/test_tools.h"

namespace TestHBReplay {
static const int TEST_APPID = 1216230;
TEST_SUITE("[HBReplay]") {
	TEST_CASE("[HBReplay] Basic format serialization") {
		Ref<HBReplayWriter> replay_data;
		replay_data.instantiate();
		replay_data->set_song_id("song_id");
		replay_data->set_song_difficulty("difficulty");
		replay_data->set_song_chart_hash("hash");
		PackedByteArray buff = replay_data->write_to_buffer();

		Vector<uint8_t> magic;
		magic.push_back('P');
		magic.push_back('H');
		magic.push_back('R');

		CHECK_MESSAGE(buff.slice(0, 3) == magic, "Replay should contain magic number.");
		CHECK_MESSAGE(buff[3] == HBReplayWriter::VERSION, "Replay should contain matching version number.");
		CHECK_MESSAGE(buff.slice(0x8, 0xF) == String("song_id").to_utf8_buffer(), "Replay should contain matching song id.");
		CHECK_MESSAGE(buff.slice(0x13, 0x1D) == String("difficulty").to_utf8_buffer(), "Replay should contain matching difficulty.");
		CHECK_MESSAGE(buff.slice(0x21, 0x25) == String("hash").to_utf8_buffer(), "Replay should contain matching hash.");
	}

	static Ref<HBReplayWriter> give_me_a_bunch_of_events(StringName p_guid1, StringName p_guid2) {
		Ref<HBReplayWriter> replay_data;
		replay_data.instantiate();
		replay_data->set_song_id("song_id");
		replay_data->set_song_difficulty("difficulty");
		replay_data->set_song_chart_hash("hash");

		constexpr int64_t game_timestamp = 6969;

		replay_data->begin_frame(0);

		Ref<HBReplayEvent> button_event;
		button_event.instantiate();

		button_event->set_event_type(HBReplay::EventType::GAMEPAD_BUTTON);
		button_event->set_gamepad_button_index((int)JoyButton::Y);
		button_event->set_game_timestamp(0);
		button_event->set_device_guid(p_guid1);
		button_event->set_device_name("Device Name");
		button_event->set_press_actions(HBReplay::EventActionBitfield::NOTE_DOWN_B | HBReplay::EventActionBitfield::NOTE_UP_B);
		button_event->set_release_actions(HBReplay::EventActionBitfield::HEART_NOTE_B);
		button_event->set_gamepad_button_pressed(true);

		replay_data->push_event(button_event);

		Ref<HBReplayEvent> joy_single_axis_event;
		joy_single_axis_event.instantiate();

		replay_data->begin_frame(game_timestamp);
		joy_single_axis_event->set_event_type(HBReplay::EventType::GAMEPAD_JOY_SINGLE_AXIS);
		joy_single_axis_event->set_joystick_axis(0, JoyAxis::TRIGGER_LEFT);
		joy_single_axis_event->set_joystick_position(Vector2(0.5, 0.0));
		joy_single_axis_event->set_game_timestamp(game_timestamp);
		joy_single_axis_event->set_device_guid(p_guid2);
		joy_single_axis_event->set_device_name("Second Device");
		joy_single_axis_event->set_press_actions(HBReplay::EventActionBitfield::NOTE_DOWN_B | HBReplay::EventActionBitfield::NOTE_UP_B);
		joy_single_axis_event->set_release_actions(HBReplay::EventActionBitfield::HEART_NOTE_B);
		replay_data->push_event(joy_single_axis_event);

		return replay_data;
	}

	TEST_CASE("[HBReplay] Gamepad info serialization") {
		Ref<HBReplayWriter> replay_data = give_me_a_bunch_of_events("GUID", "GUID2");
		PackedByteArray buff = replay_data->write_to_buffer();

		Ref<HBReplayReader> reader = HBReplayReader::from_buffer(buff);

		// Gamepads

		CHECK_MESSAGE(reader->get_gamepad_info_count() == 2, "Replay should contain two gamepad infos.");
		CHECK_MESSAGE(reader->get_gamepad_name(0) == "Device Name", "Replay should contain first device name.");
		CHECK_MESSAGE(reader->get_gamepad_guid(0) == "GUID", "Replay should contain first device GUID.");
		CHECK_MESSAGE(reader->get_gamepad_name(1) == "Second Device", "Replay should contain first device name.");
		CHECK_MESSAGE(reader->get_gamepad_guid(1) == "GUID2", "Replay should contain first device GUID.");
	}

	TEST_CASE("[HBReplay] Test getting events in interval") {
		Ref<HBReplayWriter> replay_data = give_me_a_bunch_of_events("GUID3", "GUID4");
		PackedByteArray buff = replay_data->write_to_buffer();

		Ref<HBReplayReader> reader = HBReplayReader::from_buffer(buff);

		// Gamepads

		Vector<Ref<HBReplayEvent>> events = reader->get_replay_events_in_interval(0, 6969);

		for (Ref<HBReplayEvent> event : events) {
			print_line("GS", event->get_game_timestamp());
		}

		CHECK_MESSAGE(events.size() == 1, "Getting replay events in an interval should be end exclusive");
		CHECK_MESSAGE(events[0]->get_event_type() == HBReplay::EventType::GAMEPAD_BUTTON, "Getting replay events in an interval should return the correct event");
		CHECK_MESSAGE(events[0]->get_gamepad_button_pressed(), "Test event should be pressed");

		events = reader->get_replay_events_in_interval(6969, 6970);
		CHECK_MESSAGE(events.size() == 1, "Getting replay events in an interval should be end exclusive (part 2)");
		CHECK_MESSAGE(events[0]->get_event_type() == HBReplay::EventType::GAMEPAD_JOY_SINGLE_AXIS, "Getting replay events in an interval should return the correct event (part 2)");
		CHECK_MESSAGE(events[0]->get_joystick_axis(0), "Test event's axis should be TRIGGER_LEFT");
	}

	TEST_CASE("[HBReplay] Test keyboard event serialization") {
		Ref<HBReplayWriter> replay_data = give_me_a_bunch_of_events("GUID5", "GUID6");

		Ref<HBReplayEvent> event;
		event.instantiate();
		event->set_event_type(HBReplay::EventType::KEYBOARD_KEY);
		event->set_keyboard_key((uint64_t)Key::H);
		event->set_keyboard_key_pressed(true);
		event->set_game_timestamp(70000);
		replay_data->begin_frame(70000);
		replay_data->push_event(event);

		PackedByteArray buff = replay_data->write_to_buffer();

		Ref<HBReplayReader> reader = HBReplayReader::from_buffer(buff);

		Vector<Ref<HBReplayEvent>> events = reader->get_replay_events_in_interval(70000, 70001);

		CHECK(events.size() == 1);
		CHECK(events[0]->get_event_type() == HBReplay::EventType::KEYBOARD_KEY);
		CHECK(events[0]->get_keyboard_key() == (uint64_t)Key::H);
		CHECK(events[0]->get_keyboard_key_pressed());
	}

	TEST_CASE("[HBReplay] Test gamepad axis event") {
		Ref<HBReplayWriter> replay_data = give_me_a_bunch_of_events("GUID7", "GUID8");

		Ref<HBReplayEvent> event;
		event.instantiate();
		event->set_event_type(HBReplay::EventType::GAMEPAD_JOY);
		event->set_device_guid("GUID8");
		event->set_joystick_axis(0, JoyAxis::RIGHT_X);
		event->set_joystick_axis(1, JoyAxis::RIGHT_Y);
		event->set_press_actions(HBReplay::EventActionBitfield::SLIDE_LEFT_B | HBReplay::EventActionBitfield::HEART_NOTE_B);
		event->set_joystick_position(Vector2(0.5, 0.75));
		event->set_game_timestamp(70000);
		replay_data->begin_frame(70000);
		replay_data->push_event(event);

		PackedByteArray buff = replay_data->write_to_buffer();

		Ref<HBReplayReader> reader = HBReplayReader::from_buffer(buff);

		Vector<Ref<HBReplayEvent>> events = reader->get_replay_events_in_interval(70000, 70001);

		CHECK(events.size() == 1);
		CHECK(events[0]->get_event_type() == HBReplay::EventType::GAMEPAD_JOY);
		CHECK(events[0]->get_joystick_axis(0) == (uint8_t)JoyAxis::RIGHT_X);
		CHECK(events[0]->get_joystick_axis(1) == (uint8_t)JoyAxis::RIGHT_Y);
		CHECK(events[0]->get_joystick_position() == Vector2(0.5, 0.75));
		CHECK(events[0]->get_device_guid() == "GUID8");
		CHECK(events[0]->get_device_name() == "Second Device");
		CHECK((uint8_t)(events[0]->get_press_actions()) == (uint8_t)(HBReplay::EventActionBitfield::SLIDE_LEFT_B | HBReplay::EventActionBitfield::HEART_NOTE_B));
		HBReplay::StateSnapshot snapshot{};
		events[0]->get_state_snapshot(&snapshot);
		CHECK(snapshot.action_held_count[HBReplay::EventAction::HEART_NOTE] == 1);
	}

	TEST_CASE("[HBReplay] Test gamepad button event") {
		Ref<HBReplayWriter> replay_data = give_me_a_bunch_of_events("GUID9", "GUID10");

		Ref<HBReplayEvent> event;
		event.instantiate();
		event->set_event_type(HBReplay::EventType::GAMEPAD_BUTTON);
		event->set_device_guid("GUID10");
		event->set_gamepad_button_index((uint8_t)JoyButton::DPAD_RIGHT);
		event->set_game_timestamp(70000);
		event->set_press_actions(HBReplay::EventActionBitfield::NOTE_UP_B);
		replay_data->begin_frame(70000);
		replay_data->push_event(event);

		PackedByteArray buff = replay_data->write_to_buffer();

		Ref<HBReplayReader> reader = HBReplayReader::from_buffer(buff);

		Vector<Ref<HBReplayEvent>> events = reader->get_replay_events_in_interval(70000, 70001);

		CHECK(events.size() == 1);
		CHECK(events[0]->get_event_type() == HBReplay::EventType::GAMEPAD_BUTTON);
		CHECK(events[0]->get_gamepad_button_index() == (uint8_t)JoyButton::DPAD_RIGHT);
		CHECK(events[0]->get_device_guid() == "GUID10");
	}

	TEST_CASE("[HBReplay] Test state snapshots") {
		Ref<HBReplayWriter> replay_data = give_me_a_bunch_of_events("GUID11", "GUID12");

		Ref<HBReplayEvent> event;
		event.instantiate();
		event->set_event_type(HBReplay::EventType::GAMEPAD_BUTTON);
		event->set_device_guid("GUID12");
		event->set_gamepad_button_index((uint8_t)JoyButton::DPAD_RIGHT);
		event->set_game_timestamp(70000);
		event->set_press_actions(HBReplay::EventActionBitfield::NOTE_UP_B);
		replay_data->begin_frame(70000);
		replay_data->push_event(event);

		Ref<HBReplayEvent> event2;
		event2.instantiate();
		event2->set_event_type(HBReplay::EventType::GAMEPAD_BUTTON);
		event2->set_device_guid("GUID12");
		event2->set_gamepad_button_index((uint8_t)JoyButton::DPAD_RIGHT);
		event2->set_game_timestamp(70002);
		event2->set_release_actions(HBReplay::EventActionBitfield::NOTE_UP_B);
		replay_data->begin_frame(70002);
		replay_data->push_event(event2);

		PackedByteArray buff = replay_data->write_to_buffer();

		Ref<HBReplayReader> reader = HBReplayReader::from_buffer(buff);

		Vector<Ref<HBReplayEvent>> events = reader->get_replay_events_in_interval(70000, 70003);

		CHECK(events.size() == 2);
		CHECK(events[0]->get_event_type() == HBReplay::EventType::GAMEPAD_BUTTON);
		CHECK(events[0]->get_gamepad_button_index() == (uint8_t)JoyButton::DPAD_RIGHT);
		CHECK(events[0]->get_device_guid() == "GUID12");

		HBReplay::StateSnapshot state_snapshot;
		events[0]->get_state_snapshot(&state_snapshot);
		CHECK(state_snapshot.action_held_count[HBReplay::EventAction::NOTE_UP] == 3);

		events[1]->get_state_snapshot(&state_snapshot);
		CHECK(state_snapshot.action_held_count[HBReplay::EventAction::NOTE_UP] == 2);

		Ref<FileAccess> fa = FileAccess::open("/home/eirexe/repos/godot-ph4/bin/replay.phr", FileAccess::WRITE);
		fa->store_buffer(buff);
		fa->close();
	}
}

} //namespace TestHBReplay

#endif // TESTS_HB_REPLAY_H
