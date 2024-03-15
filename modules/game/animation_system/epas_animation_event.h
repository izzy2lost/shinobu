/**************************************************************************/
/*  epas_animation_event.h                                                */
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

#ifndef EPAS_ANIMATION_EVENT_H
#define EPAS_ANIMATION_EVENT_H

#include "core/io/resource.h"

class AudioStream;

class EPASAnimationEvent : public Resource {
	GDCLASS(EPASAnimationEvent, Resource);
	float time = 0.0f;

protected:
	static void _bind_methods();

public:
	float get_time() const;
	void set_time(float p_time);
};

class EPASSoundAnimationEvent : public EPASAnimationEvent {
	GDCLASS(EPASSoundAnimationEvent, EPASAnimationEvent);

private:
	Ref<AudioStream> stream;

protected:
	static void _bind_methods();

public:
	Ref<AudioStream> get_stream() const;
	void set_stream(const Ref<AudioStream> &p_stream);
};

#endif // EPAS_ANIMATION_EVENT_H
