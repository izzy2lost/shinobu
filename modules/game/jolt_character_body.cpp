#include "jolt_character_body.h"

#include "modules/game/physics_layers.h"
#include "modules/jolt/src/objects/jolt_body_impl_3d.hpp"
#include "modules/jolt/src/precompiled.hpp"
#include "modules/jolt/src/servers/jolt_physics_server_3d.hpp"
#include "modules/jolt/src/spaces/jolt_physics_direct_space_state_3d.hpp"
#include "modules/jolt/src/spaces/jolt_query_filter_3d.hpp"
#include "modules/jolt/src/spaces/jolt_space_3d.hpp"
#include "scene/3d/collision_shape_3d.h"
#include "scene/resources/capsule_shape_3d.h"
#include "spaces/jolt_temp_allocator.hpp"
JoltCharacterBody3D::JoltCharacterBody3D() :
		PhysicsBody3D(PhysicsServer3D::BODY_MODE_KINEMATIC) {
	if (!Engine::get_singleton()->is_editor_hint()) {
		set_physics_process(true);
	}
	temp_allocator = new JoltTempAllocator();
};

JoltCharacterBody3D::~JoltCharacterBody3D() {
	delete_safely(temp_allocator);
}

JPH::Ref<JPH::CharacterVirtualSettings> JoltCharacterBody3D::get_settings() const {
	JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
	static constexpr float character_height = 1.35f;
	static constexpr float character_radius = 0.3f;
	settings->mShape = JPH::RotatedTranslatedShapeSettings(JPH::Vec3(0, 0.5f * character_height + character_radius, 0), JPH::Quat::sIdentity(), new JPH::CapsuleShape(0.5f * character_height, character_radius)).Create().Get();
	;
	return settings;
}

void JoltCharacterBody3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			Ref<CapsuleShape3D> capsule;
			capsule.instantiate();
			static constexpr float character_height = 1.35f;
			static constexpr float character_radius = 0.3f;
			capsule->set_height(character_height);
			capsule->set_radius(character_radius);
			CollisionShape3D *shape = memnew(CollisionShape3D);
			add_child(shape);
			shape->set_position(Vector3(0.0f, 0.5f * character_height + character_radius, 0.0f));
			shape->set_owner(get_owner());
			shape->set_shape(capsule);
			set_collision_layer(0);
		} break;
		case NOTIFICATION_ENTER_WORLD: {
			character = nullptr;

			JoltPhysicsServer3D *jolt_server = Object::cast_to<JoltPhysicsServer3D>(PhysicsServer3D::get_singleton());
			JPH::PhysicsSystem &ps = jolt_server->get_space(get_world_3d()->get_space())->get_physics_system();
			set_axis_lock(PhysicsServer3D::BODY_AXIS_ANGULAR_X, true);
			set_axis_lock(PhysicsServer3D::BODY_AXIS_ANGULAR_Y, true);
			set_axis_lock(PhysicsServer3D::BODY_AXIS_ANGULAR_Z, true);

			JPH::Vec3 pos = to_jolt(get_global_position());
			JPH::Ref<JPH::CharacterVirtualSettings> char_settings = get_settings();
			const JPH::CharacterVirtualSettings *settings = char_settings.GetPtr();
			character = new JPH::CharacterVirtual(settings, pos, JPH::Quat::sIdentity(), &ps);
			character->SetListener(this);
		} break;
		case NOTIFICATION_PHYSICS_PROCESS: {
		} break;
	}
}

void JoltCharacterBody3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_velocity"), &JoltCharacterBody3D::get_velocity);
	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &JoltCharacterBody3D::set_velocity);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity"), "set_velocity", "get_velocity");
	ClassDB::bind_method(D_METHOD("get_effective_velocity"), &JoltCharacterBody3D::get_effective_velocity);
	GDVIRTUAL_BIND(_post_physics_process, "delta");
	ClassDB::bind_method(D_METHOD("get_desired_velocity"), &JoltCharacterBody3D::get_desired_velocity);
	ClassDB::bind_method(D_METHOD("get_ground_velocity"), &JoltCharacterBody3D::get_ground_velocity);
}

void JoltCharacterBody3D::OnContactSolve(const JPH::CharacterVirtual *inCharacter, const JPH::BodyID &inBodyID2, const JPH::SubShapeID &inSubShapeID2, JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::Vec3Arg inContactVelocity, const JPH::PhysicsMaterial *inContactMaterial, JPH::Vec3Arg inCharacterVelocity, JPH::Vec3 &ioNewCharacterVelocity) {
	// Don't allow the player to slide down static not-too-steep surfaces when not actively moving and when not on a moving platform
	if (!allow_sliding && inContactVelocity.IsNearZero() && !inCharacter->IsSlopeTooSteep(inContactNormal))
		ioNewCharacterVelocity = JPH::Vec3::sZero();
}

Vector3 JoltCharacterBody3D::get_linear_velocity() const {
	ERR_FAIL_COND_V(!is_inside_tree(), Vector3());
	return to_godot(character->GetLinearVelocity());
}

Vector3 JoltCharacterBody3D::get_velocity() const {
	return get_linear_velocity();
}

void JoltCharacterBody3D::set_velocity(Vector3 p_velocity) {
	set_linear_velocity(p_velocity);
}

void JoltCharacterBody3D::set_linear_velocity(Vector3 p_velocity) {
	ERR_FAIL_COND(!is_inside_tree());
	character->SetLinearVelocity(to_jolt(p_velocity));
}

Vector3 JoltCharacterBody3D::get_effective_velocity() const {
	return effective_velocity;
}

void JoltCharacterBody3D::handle_input(Vector3 p_input, float p_delta) {
	bool player_controls_horizontal_velocity = character->IsSupported();
	if (player_controls_horizontal_velocity) {
		// True if the player intended to move
		allow_sliding = !p_input.is_zero_approx();
	} else {
		// While in air we allow sliding
		allow_sliding = true;
	}

	// A cheaper way to update the character's ground velocity,
	// the platforms that the character is standing on may have changed velocity
	character->UpdateGroundVelocity();

	// Determine new basic velocity
	JPH::Vec3 current_vertical_velocity = character->GetLinearVelocity().Dot(character->GetUp()) * character->GetUp();
	JPH::Vec3 ground_velocity = character->GetGroundVelocity();
	JPH::Vec3 new_velocity;
	bool moving_towards_ground = (current_vertical_velocity.GetY() - ground_velocity.GetY()) < 0.1f;
	if (character->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround // If on ground
			&& (inertia_enabled ? moving_towards_ground // Inertia enabled: And not moving away from ground
								: !character->IsSlopeTooSteep(character->GetGroundNormal()))) // Inertia disabled: And not on a slope that is too steep
	{
		// Assume velocity of ground when on ground
		new_velocity = ground_velocity;
	} else
		new_velocity = current_vertical_velocity;

	// Gravity
	new_velocity += JPH::Vec3(0.0f, gravity, 0.0f) * p_delta;

	if (player_controls_horizontal_velocity) {
		// Player input
		new_velocity += to_jolt(desired_velocity);
	} else {
		// Preserve horizontal velocity
		JPH::Vec3 current_horizontal_velocity = character->GetLinearVelocity() - current_vertical_velocity;
		new_velocity += current_horizontal_velocity;
	}

	// Update character velocity
	character->SetLinearVelocity(new_velocity);
}

float JoltCharacterBody3D::get_floor_max_angle() const {
	return Math::deg_to_rad(55.0f);
}

bool JoltCharacterBody3D::is_on_floor() const {
	return character->IsSupported();
}

void JoltCharacterBody3D::update(float p_delta) {
	character->SetPosition(to_jolt(get_global_position()));
	Vector3 old_position = to_godot(character->GetPosition());
	// Settings for our update function
	JPH::CharacterVirtual::ExtendedUpdateSettings update_settings;
	if (!stick_to_floor_enabled)
		update_settings.mStickToFloorStepDown = JPH::Vec3::sZero();
	else
		update_settings.mStickToFloorStepDown = -character->GetUp() * update_settings.mStickToFloorStepDown.Length();
	if (!walk_stairs_enabled)
		update_settings.mWalkStairsStepUp = JPH::Vec3::sZero();
	else
		update_settings.mWalkStairsStepUp = character->GetUp() * update_settings.mWalkStairsStepUp.Length();

	// Get jolt physics system from godot
	JoltPhysicsServer3D *jolt_server = Object::cast_to<JoltPhysicsServer3D>(PhysicsServer3D::get_singleton());
	const RID space = get_world_3d()->get_space();
	JoltPhysicsDirectSpaceState3D *dss = Object::cast_to<JoltPhysicsDirectSpaceState3D>(jolt_server->space_get_direct_state(space));
	const JoltQueryFilter3D query_filter(*dss, get_collision_mask(), true, false);

	// Update the character position
	character->ExtendedUpdate(p_delta,
			JPH::Vec3(0.0f, gravity, 0.0f),
			update_settings,
			query_filter,
			query_filter,
			{},
			{},
			*temp_allocator);

	// Update our position
	set_global_position(to_godot(character->GetPosition()));

	// Calculate effective velocity
	Vector3 new_position = to_godot(character->GetPosition());
	effective_velocity = (new_position - old_position) / p_delta;
	GDVIRTUAL_CALL(_post_physics_process, p_delta);
}

Vector3 JoltCharacterBody3D::get_ground_velocity() const {
	ERR_FAIL_NULL_V(character.GetPtr(), Vector3());
	return to_godot(character->GetGroundVelocity());
}