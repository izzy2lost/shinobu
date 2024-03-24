/**************************************************************************/
/*  decal_ex.cpp                                                          */
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

#include "decal_ex.h"

void DecalEX::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_alpha_clip_threshold_max"), &DecalEX::get_alpha_clip_threshold_max);
	ClassDB::bind_method(D_METHOD("set_alpha_clip_threshold_max", "alpha_clip_threshold_max"), &DecalEX::set_alpha_clip_threshold_max);

	ClassDB::bind_method(D_METHOD("get_alpha_clip_threshold_min"), &DecalEX::get_alpha_clip_threshold_min);
	ClassDB::bind_method(D_METHOD("set_alpha_clip_threshold_min", "alpha_clip_threshold_min"), &DecalEX::set_alpha_clip_threshold_min);

	ADD_GROUP("Alpha Clip", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alpha_clip_threshold_min"), "set_alpha_clip_threshold_min", "get_alpha_clip_threshold_min");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "alpha_clip_threshold_max"), "set_alpha_clip_threshold_max", "get_alpha_clip_threshold_max");
}

float DecalEX::get_alpha_clip_threshold_min() const {
	return alpha_clip_threshold_min;
}

void DecalEX::set_alpha_clip_threshold_min(float p_alpha_clip_threshold_low) {
	alpha_clip_threshold_min = CLAMP(p_alpha_clip_threshold_low, 0.0, 1.0);
	RID decal_rid = get_rid();
	RS::get_singleton()->decal_set_alpha_clip_threshold(decal_rid, alpha_clip_threshold_min, alpha_clip_threshold_max);
}

float DecalEX::get_alpha_clip_threshold_max() const {
	return alpha_clip_threshold_max;
}

void DecalEX::set_alpha_clip_threshold_max(float p_alpha_clip_threshold_high) {
	alpha_clip_threshold_max = CLAMP(p_alpha_clip_threshold_high, 0.0, 1.0);
	RID decal_rid = get_rid();
	RS::get_singleton()->decal_set_alpha_clip_threshold(decal_rid, alpha_clip_threshold_min, alpha_clip_threshold_max);
}
