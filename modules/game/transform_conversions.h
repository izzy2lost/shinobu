/**************************************************************************/
/*  transform_conversions.h                                               */
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

#ifndef TRANSFORM_CONVERSIONS_H
#define TRANSFORM_CONVERSIONS_H

#include "core/math/projection.h"
#include "core/math/transform_3d.h"

class HBTransformConversions {
public:
	static void trf_to_mat(const Transform3D &p_mat, float *p_out) {
		p_out[0] = p_mat.basis.rows[0][0];
		p_out[1] = p_mat.basis.rows[1][0];
		p_out[2] = p_mat.basis.rows[2][0];
		p_out[3] = 0;

		p_out[4] = p_mat.basis.rows[0][1];
		p_out[5] = p_mat.basis.rows[1][1];
		p_out[6] = p_mat.basis.rows[2][1];
		p_out[7] = 0;

		p_out[8] = p_mat.basis.rows[0][2];
		p_out[9] = p_mat.basis.rows[1][2];
		p_out[10] = p_mat.basis.rows[2][2];
		p_out[11] = 0;

		p_out[12] = p_mat.origin.x;
		p_out[13] = p_mat.origin.y;
		p_out[14] = p_mat.origin.z;
		p_out[15] = 1;
	}

	static void mat_to_trf(const float *p_in, Transform3D &p_out) {
		p_out.basis.rows[0][0] = p_in[0];
		p_out.basis.rows[1][0] = p_in[1];
		p_out.basis.rows[2][0] = p_in[2];

		p_out.basis.rows[0][1] = p_in[4];
		p_out.basis.rows[1][1] = p_in[5];
		p_out.basis.rows[2][1] = p_in[6];

		p_out.basis.rows[0][2] = p_in[8];
		p_out.basis.rows[1][2] = p_in[9];
		p_out.basis.rows[2][2] = p_in[10];

		p_out.origin.x = p_in[12];
		p_out.origin.y = p_in[13];
		p_out.origin.z = p_in[14];
	}

	static void proj_to_mat(const Projection &p_mat, float *p_out) {
		for (int i = 0; i < 16; i++) {
			real_t *p = (real_t *)p_mat.columns;
			p_out[i] = p[i];
		}
	}
};

#endif // TRANSFORM_CONVERSIONS_H
