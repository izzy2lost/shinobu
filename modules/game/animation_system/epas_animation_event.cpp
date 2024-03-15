/**************************************************************************/
/*  epas_animation_event.cpp                                              */
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

#include "epas_animation_event.h"

#include "servers/audio/audio_stream.h"

void EPASSoundAnimationEvent::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_stream", "stream"), &EPASSoundAnimationEvent::set_stream);
	ClassDB::bind_method(D_METHOD("get_stream"), &EPASSoundAnimationEvent::get_stream);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stream", PROPERTY_HINT_RESOURCE_TYPE, "AudioStream"), "set_stream", "get_stream");

	ClassDB::bind_method(D_METHOD("set_time", "time"), &EPASSoundAnimationEvent::set_time);
	ClassDB::bind_method(D_METHOD("get_time"), &EPASSoundAnimationEvent::get_time);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time"), "set_time", "get_time");
}

Ref<AudioStream> EPASSoundAnimationEvent::get_stream() const {
	return stream;
}

void EPASSoundAnimationEvent::set_stream(const Ref<AudioStream> &p_stream) {
	stream = p_stream;
}

void EPASAnimationEvent::_bind_methods() {
	ADD_SIGNAL(MethodInfo("time_changed"));
}

float EPASAnimationEvent::get_time() const {
	return time;
}

void EPASAnimationEvent::set_time(float p_time) {
	time = p_time;
	emit_signal("time_changed");
}
