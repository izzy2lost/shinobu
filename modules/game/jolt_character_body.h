/**************************************************************************/
/*  jolt_character_body.h                                                 */
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

#ifndef JOLT_TEST_H
#define JOLT_TEST_H

// Make sure clang format doesn't reorder our headers
/* clang-format off */
#include "modules/jolt/src/precompiled.hpp"
#include "Jolt/Physics/Character/CharacterVirtual.h"
/* clang-format on */

class JoltCharacterBody3D : public PhysicsBody3D, public JPH::CharacterContactListener {
	GDCLASS(JoltCharacterBody3D, PhysicsBody3D);

	bool inertia_enabled = true;
	bool stick_to_floor_enabled = true;
	bool walk_stairs_enabled = true;
	float gravity = -20.0f;
	bool allow_sliding = false;
	Vector3 desired_velocity;
	Vector3 effective_velocity;

	RID body;
	JPH::Ref<JPH::CharacterVirtual> character;
	JPH::TempAllocator *temp_allocator;

	JPH::Ref<JPH::CharacterVirtualSettings> get_settings() const;

protected:
	void _notification(int p_what);
	static void _bind_methods();
	GDVIRTUAL1(_post_physics_process, double)
	virtual void OnContactSolve(const JPH::CharacterVirtual *inCharacter, const JPH::BodyID &inBodyID2, const JPH::SubShapeID &inSubShapeID2, JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::Vec3Arg inContactVelocity, const JPH::PhysicsMaterial *inContactMaterial, JPH::Vec3Arg inCharacterVelocity, JPH::Vec3 &ioNewCharacterVelocity) override;

public:
	virtual Vector3 get_linear_velocity() const override;
	Vector3 get_velocity() const;
	void set_velocity(Vector3 p_velocity);
	void set_linear_velocity(Vector3 p_velocity);
	Vector3 get_effective_velocity() const;
	void handle_input(Vector3 p_input, float p_delta);
	float get_floor_max_angle() const;
	bool is_on_floor() const;
	JPH::CharacterVirtual::EGroundState get_ground_state() const;
	virtual void update(float p_delta);
	Vector3 get_walk_stairs_step_up() const;
	JoltCharacterBody3D();
	~JoltCharacterBody3D();

	Vector3 get_ground_velocity() const;

	Vector3 get_desired_velocity() const { return desired_velocity; }
	void set_desired_velocity(const Vector3 &p_desired_velocity) { desired_velocity = p_desired_velocity; }
};

#endif // JOLT_TEST_H
