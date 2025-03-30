#nullable enable
#pragma warning disable CS9084
using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Box2D.NET.Bindings;
public static unsafe partial class B2
{
    public partial class BindgenInternal
    {
        public const string DllImportPath = @"box2d";
    }

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Center")]
    public static extern Vec2 AABBCenter(AABB a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Contains")]
    public static extern byte AABBContains(AABB a, AABB b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Extents")]
    public static extern Vec2 AABBExtents(AABB a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Union")]
    public static extern AABB AABBUnion(AABB a, AABB b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Abs")]
    public static extern Vec2 Abs(Vec2 a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AbsFloat")]
    public static extern float AbsFloat(float a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AbsInt")]
    public static extern int AbsInt(int a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Add")]
    public static extern Vec2 Add(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Atan2")]
    public static extern float Atan2(float y, float x);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyAngularImpulse")]
    public static extern void BodyApplyAngularImpulse(BodyId bodyId, float impulse, byte wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyForce")]
    public static extern void BodyApplyForce(BodyId bodyId, Vec2 force, Vec2 point, byte wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyForceToCenter")]
    public static extern void BodyApplyForceToCenter(BodyId bodyId, Vec2 force, byte wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyLinearImpulse")]
    public static extern void BodyApplyLinearImpulse(BodyId bodyId, Vec2 impulse, Vec2 point, byte wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyLinearImpulseToCenter")]
    public static extern void BodyApplyLinearImpulseToCenter(BodyId bodyId, Vec2 impulse, byte wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyMassFromShapes")]
    public static extern void BodyApplyMassFromShapes(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyTorque")]
    public static extern void BodyApplyTorque(BodyId bodyId, float torque, byte wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ComputeAABB")]
    public static extern AABB BodyComputeAABB(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Disable")]
    public static extern void BodyDisable(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Enable")]
    public static extern void BodyEnable(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableContactEvents")]
    public static extern void BodyEnableContactEvents(BodyId bodyId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableHitEvents")]
    public static extern void BodyEnableHitEvents(BodyId bodyId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableSleep")]
    public static extern void BodyEnableSleep(BodyId bodyId, byte enableSleep);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularDamping")]
    public static extern float BodyGetAngularDamping(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularVelocity")]
    public static extern float BodyGetAngularVelocity(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactCapacity")]
    public static extern int BodyGetContactCapacity(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactData")]
    public static extern int BodyGetContactData(BodyId bodyId, ContactData* contactData, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetGravityScale")]
    public static extern float BodyGetGravityScale(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetJointCount")]
    public static extern int BodyGetJointCount(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetJoints")]
    public static extern int BodyGetJoints(BodyId bodyId, JointId* jointArray, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLinearDamping")]
    public static extern float BodyGetLinearDamping(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLinearVelocity")]
    public static extern Vec2 BodyGetLinearVelocity(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalCenterOfMass")]
    public static extern Vec2 BodyGetLocalCenterOfMass(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalPoint")]
    public static extern Vec2 BodyGetLocalPoint(BodyId bodyId, Vec2 worldPoint);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalPointVelocity")]
    public static extern Vec2 BodyGetLocalPointVelocity(BodyId bodyId, Vec2 localPoint);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalVector")]
    public static extern Vec2 BodyGetLocalVector(BodyId bodyId, Vec2 worldVector);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetMass")]
    public static extern float BodyGetMass(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetMassData")]
    public static extern MassData BodyGetMassData(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetName")]
    public static extern byte* BodyGetName(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetPosition")]
    public static extern Vec2 BodyGetPosition(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetRotation")]
    public static extern Rot BodyGetRotation(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetRotationalInertia")]
    public static extern float BodyGetRotationalInertia(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetShapeCount")]
    public static extern int BodyGetShapeCount(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetShapes")]
    public static extern int BodyGetShapes(BodyId bodyId, ShapeId* shapeArray, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetSleepThreshold")]
    public static extern float BodyGetSleepThreshold(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetTransform")]
    public static extern Transform BodyGetTransform(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetType")]
    public static extern BodyType BodyGetType(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetUserData")]
    public static extern void* BodyGetUserData(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorld")]
    public static extern WorldId BodyGetWorld(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldCenterOfMass")]
    public static extern Vec2 BodyGetWorldCenterOfMass(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldPoint")]
    public static extern Vec2 BodyGetWorldPoint(BodyId bodyId, Vec2 localPoint);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldPointVelocity")]
    public static extern Vec2 BodyGetWorldPointVelocity(BodyId bodyId, Vec2 worldPoint);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldVector")]
    public static extern Vec2 BodyGetWorldVector(BodyId bodyId, Vec2 localVector);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsAwake")]
    public static extern byte BodyIsAwake(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsBullet")]
    public static extern byte BodyIsBullet(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsEnabled")]
    public static extern byte BodyIsEnabled(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsFixedRotation")]
    public static extern byte BodyIsFixedRotation(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsSleepEnabled")]
    public static extern byte BodyIsSleepEnabled(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsValid")]
    public static extern byte BodyIsValid(BodyId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularDamping")]
    public static extern void BodySetAngularDamping(BodyId bodyId, float angularDamping);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularVelocity")]
    public static extern void BodySetAngularVelocity(BodyId bodyId, float angularVelocity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAwake")]
    public static extern void BodySetAwake(BodyId bodyId, byte awake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetBullet")]
    public static extern void BodySetBullet(BodyId bodyId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetFixedRotation")]
    public static extern void BodySetFixedRotation(BodyId bodyId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetGravityScale")]
    public static extern void BodySetGravityScale(BodyId bodyId, float gravityScale);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetLinearDamping")]
    public static extern void BodySetLinearDamping(BodyId bodyId, float linearDamping);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetLinearVelocity")]
    public static extern void BodySetLinearVelocity(BodyId bodyId, Vec2 linearVelocity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetMassData")]
    public static extern void BodySetMassData(BodyId bodyId, MassData massData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetName")]
    public static extern void BodySetName(BodyId bodyId, byte* name);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetSleepThreshold")]
    public static extern void BodySetSleepThreshold(BodyId bodyId, float sleepThreshold);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetTransform")]
    public static extern void BodySetTransform(BodyId bodyId, Vec2 position, Rot rotation);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetType")]
    public static extern void BodySetType(BodyId bodyId, BodyType type);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetUserData")]
    public static extern void BodySetUserData(BodyId bodyId, void* userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_GetFriction")]
    public static extern float ChainGetFriction(ChainId chainId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_GetMaterial")]
    public static extern int ChainGetMaterial(ChainId chainId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_GetRestitution")]
    public static extern float ChainGetRestitution(ChainId chainId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_GetSegmentCount")]
    public static extern int ChainGetSegmentCount(ChainId chainId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_GetSegments")]
    public static extern int ChainGetSegments(ChainId chainId, ShapeId* segmentArray, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_GetWorld")]
    public static extern WorldId ChainGetWorld(ChainId chainId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_IsValid")]
    public static extern byte ChainIsValid(ChainId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetFriction")]
    public static extern void ChainSetFriction(ChainId chainId, float friction);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetMaterial")]
    public static extern void ChainSetMaterial(ChainId chainId, int material);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetRestitution")]
    public static extern void ChainSetRestitution(ChainId chainId, float restitution);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Clamp")]
    public static extern Vec2 Clamp(Vec2 v, Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ClampFloat")]
    public static extern float ClampFloat(float a, float lower, float upper);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ClampInt")]
    public static extern int ClampInt(int a, int lower, int upper);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCapsuleAndCircle")]
    public static extern Manifold CollideCapsuleAndCircle(Capsule* capsuleA, Transform xfA, Circle* circleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCapsules")]
    public static extern Manifold CollideCapsules(Capsule* capsuleA, Transform xfA, Capsule* capsuleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideChainSegmentAndCapsule")]
    public static extern Manifold CollideChainSegmentAndCapsule(ChainSegment* segmentA, Transform xfA, Capsule* capsuleB, Transform xfB, SimplexCache* cache);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideChainSegmentAndCircle")]
    public static extern Manifold CollideChainSegmentAndCircle(ChainSegment* segmentA, Transform xfA, Circle* circleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideChainSegmentAndPolygon")]
    public static extern Manifold CollideChainSegmentAndPolygon(ChainSegment* segmentA, Transform xfA, Polygon* polygonB, Transform xfB, SimplexCache* cache);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCircles")]
    public static extern Manifold CollideCircles(Circle* circleA, Transform xfA, Circle* circleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygonAndCapsule")]
    public static extern Manifold CollidePolygonAndCapsule(Polygon* polygonA, Transform xfA, Capsule* capsuleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygonAndCircle")]
    public static extern Manifold CollidePolygonAndCircle(Polygon* polygonA, Transform xfA, Circle* circleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygons")]
    public static extern Manifold CollidePolygons(Polygon* polygonA, Transform xfA, Polygon* polygonB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndCapsule")]
    public static extern Manifold CollideSegmentAndCapsule(Segment* segmentA, Transform xfA, Capsule* capsuleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndCircle")]
    public static extern Manifold CollideSegmentAndCircle(Segment* segmentA, Transform xfA, Circle* circleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndPolygon")]
    public static extern Manifold CollideSegmentAndPolygon(Segment* segmentA, Transform xfA, Polygon* polygonB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeAngularVelocity")]
    public static extern float ComputeAngularVelocity(Rot q1, Rot q2, float inv_h);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCapsuleAABB")]
    public static extern AABB ComputeCapsuleAABB(Capsule* shape, Transform transform);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCapsuleMass")]
    public static extern MassData ComputeCapsuleMass(Capsule* shape, float density);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCircleAABB")]
    public static extern AABB ComputeCircleAABB(Circle* shape, Transform transform);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCircleMass")]
    public static extern MassData ComputeCircleMass(Circle* shape, float density);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCosSin")]
    public static extern CosSin ComputeCosSin(float radians);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeHull")]
    public static extern Hull ComputeHull(Vec2* points, int count);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputePolygonAABB")]
    public static extern AABB ComputePolygonAABB(Polygon* shape, Transform transform);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputePolygonMass")]
    public static extern MassData ComputePolygonMass(Polygon* shape, float density);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeRotationBetweenUnitVectors")]
    public static extern Rot ComputeRotationBetweenUnitVectors(Vec2 v1, Vec2 v2);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeSegmentAABB")]
    public static extern AABB ComputeSegmentAABB(Segment* shape, Transform transform);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateBody")]
    public static extern BodyId CreateBody(WorldId worldId, BodyDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateCapsuleShape")]
    public static extern ShapeId CreateCapsuleShape(BodyId bodyId, ShapeDef* def, Capsule* capsule);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateChain")]
    public static extern ChainId CreateChain(BodyId bodyId, ChainDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateCircleShape")]
    public static extern ShapeId CreateCircleShape(BodyId bodyId, ShapeDef* def, Circle* circle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateDistanceJoint")]
    public static extern JointId CreateDistanceJoint(WorldId worldId, DistanceJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateMotorJoint")]
    public static extern JointId CreateMotorJoint(WorldId worldId, MotorJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateMouseJoint")]
    public static extern JointId CreateMouseJoint(WorldId worldId, MouseJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateNullJoint")]
    public static extern JointId CreateNullJoint(WorldId worldId, NullJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreatePolygonShape")]
    public static extern ShapeId CreatePolygonShape(BodyId bodyId, ShapeDef* def, Polygon* polygon);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreatePrismaticJoint")]
    public static extern JointId CreatePrismaticJoint(WorldId worldId, PrismaticJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateRevoluteJoint")]
    public static extern JointId CreateRevoluteJoint(WorldId worldId, RevoluteJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateSegmentShape")]
    public static extern ShapeId CreateSegmentShape(BodyId bodyId, ShapeDef* def, Segment* segment);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateWeldJoint")]
    public static extern JointId CreateWeldJoint(WorldId worldId, WeldJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateWheelJoint")]
    public static extern JointId CreateWheelJoint(WorldId worldId, WheelJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateWorld")]
    public static extern WorldId CreateWorld(WorldDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Cross")]
    public static extern float Cross(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CrossSV")]
    public static extern Vec2 CrossSV(float s, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CrossVS")]
    public static extern Vec2 CrossVS(Vec2 v, float s);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultBodyDef")]
    public static extern BodyDef DefaultBodyDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultChainDef")]
    public static extern ChainDef DefaultChainDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultDebugDraw")]
    public static extern DebugDraw DefaultDebugDraw();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultDistanceJointDef")]
    public static extern DistanceJointDef DefaultDistanceJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultExplosionDef")]
    public static extern ExplosionDef DefaultExplosionDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultFilter")]
    public static extern Filter DefaultFilter();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultMotorJointDef")]
    public static extern MotorJointDef DefaultMotorJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultMouseJointDef")]
    public static extern MouseJointDef DefaultMouseJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultNullJointDef")]
    public static extern NullJointDef DefaultNullJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultPrismaticJointDef")]
    public static extern PrismaticJointDef DefaultPrismaticJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultQueryFilter")]
    public static extern QueryFilter DefaultQueryFilter();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultRevoluteJointDef")]
    public static extern RevoluteJointDef DefaultRevoluteJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultShapeDef")]
    public static extern ShapeDef DefaultShapeDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultSurfaceMaterial")]
    public static extern SurfaceMaterial DefaultSurfaceMaterial();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultWeldJointDef")]
    public static extern WeldJointDef DefaultWeldJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultWheelJointDef")]
    public static extern WheelJointDef DefaultWheelJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultWorldDef")]
    public static extern WorldDef DefaultWorldDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyBody")]
    public static extern void DestroyBody(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyChain")]
    public static extern void DestroyChain(ChainId chainId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyJoint")]
    public static extern void DestroyJoint(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyShape")]
    public static extern void DestroyShape(ShapeId shapeId, byte updateBodyMass);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyWorld")]
    public static extern void DestroyWorld(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Distance")]
    public static extern float Distance(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableLimit")]
    public static extern void DistanceJointEnableLimit(JointId jointId, byte enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableMotor")]
    public static extern void DistanceJointEnableMotor(JointId jointId, byte enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableSpring")]
    public static extern void DistanceJointEnableSpring(JointId jointId, byte enableSpring);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetCurrentLength")]
    public static extern float DistanceJointGetCurrentLength(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetLength")]
    public static extern float DistanceJointGetLength(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMaxLength")]
    public static extern float DistanceJointGetMaxLength(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMaxMotorForce")]
    public static extern float DistanceJointGetMaxMotorForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMinLength")]
    public static extern float DistanceJointGetMinLength(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMotorForce")]
    public static extern float DistanceJointGetMotorForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMotorSpeed")]
    public static extern float DistanceJointGetMotorSpeed(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetSpringDampingRatio")]
    public static extern float DistanceJointGetSpringDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetSpringHertz")]
    public static extern float DistanceJointGetSpringHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsLimitEnabled")]
    public static extern byte DistanceJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsMotorEnabled")]
    public static extern byte DistanceJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsSpringEnabled")]
    public static extern byte DistanceJointIsSpringEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetLength")]
    public static extern void DistanceJointSetLength(JointId jointId, float length);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetLengthRange")]
    public static extern void DistanceJointSetLengthRange(JointId jointId, float minLength, float maxLength);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetMaxMotorForce")]
    public static extern void DistanceJointSetMaxMotorForce(JointId jointId, float force);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetMotorSpeed")]
    public static extern void DistanceJointSetMotorSpeed(JointId jointId, float motorSpeed);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetSpringDampingRatio")]
    public static extern void DistanceJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetSpringHertz")]
    public static extern void DistanceJointSetSpringHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceSquared")]
    public static extern float DistanceSquared(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Dot")]
    public static extern float Dot(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Create")]
    public static extern DynamicTree DynamicTreeCreate();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_CreateProxy")]
    public static extern int DynamicTreeCreateProxy(DynamicTree* tree, AABB aabb, ulong categoryBits, int userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Destroy")]
    public static extern void DynamicTreeDestroy(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_DestroyProxy")]
    public static extern void DynamicTreeDestroyProxy(DynamicTree* tree, int proxyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_EnlargeProxy")]
    public static extern void DynamicTreeEnlargeProxy(DynamicTree* tree, int proxyId, AABB aabb);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetAABB")]
    public static extern AABB DynamicTreeGetAABB(DynamicTree* tree, int proxyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetAreaRatio")]
    public static extern float DynamicTreeGetAreaRatio(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetByteCount")]
    public static extern int DynamicTreeGetByteCount(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetHeight")]
    public static extern int DynamicTreeGetHeight(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetProxyCount")]
    public static extern int DynamicTreeGetProxyCount(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetUserData")]
    public static extern int DynamicTreeGetUserData(DynamicTree* tree, int proxyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_MoveProxy")]
    public static extern void DynamicTreeMoveProxy(DynamicTree* tree, int proxyId, AABB aabb);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Query")]
    public static extern TreeStats DynamicTreeQuery(DynamicTree* tree, AABB aabb, ulong maskBits, delegate* unmanaged<int, int, void*, byte> callback, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RayCast")]
    public static extern TreeStats DynamicTreeRayCast(DynamicTree* tree, RayCastInput* input, ulong maskBits, delegate* unmanaged<RayCastInput*, int, int, void*, float> callback, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Rebuild")]
    public static extern int DynamicTreeRebuild(DynamicTree* tree, byte fullBuild);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShapeCast")]
    public static extern TreeStats DynamicTreeShapeCast(DynamicTree* tree, ShapeCastInput* input, ulong maskBits, delegate* unmanaged<ShapeCastInput*, int, int, void*, float> callback, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Validate")]
    public static extern void DynamicTreeValidate(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ValidateNoEnlarged")]
    public static extern void DynamicTreeValidateNoEnlarged(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetByteCount")]
    public static extern int GetByteCount();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetInverse22")]
    public static extern Mat22 GetInverse22(Mat22 A);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthAndNormalize")]
    public static extern Vec2 GetLengthAndNormalize(float* length, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthUnitsPerMeter")]
    public static extern float GetLengthUnitsPerMeter();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMilliseconds")]
    public static extern float GetMilliseconds(ulong ticks);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMillisecondsAndReset")]
    public static extern float GetMillisecondsAndReset(ulong* ticks);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetSweepTransform")]
    public static extern Transform GetSweepTransform(Sweep* sweep, float time);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetTicks")]
    public static extern ulong GetTicks();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetVersion")]
    public static extern Version GetVersion();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Hash")]
    public static extern uint Hash(uint hash, byte* data, int count);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IntegrateRotation")]
    public static extern Rot IntegrateRotation(Rot q1, float deltaAngle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InternalAssertFcn")]
    public static extern int InternalAssertFcn(byte* condition, byte* fileName, int lineNumber);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulRot")]
    public static extern Rot InvMulRot(Rot q, Rot r);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulTransforms")]
    public static extern Transform InvMulTransforms(Transform A, Transform B);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvRotateVector")]
    public static extern Vec2 InvRotateVector(Rot q, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvTransformPoint")]
    public static extern Vec2 InvTransformPoint(Transform t, Vec2 p);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsNormalized")]
    public static extern byte IsNormalized(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidAABB")]
    public static extern byte IsValidAABB(AABB aabb);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidFloat")]
    public static extern byte IsValidFloat(float a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidRay")]
    public static extern byte IsValidRay(RayCastInput* input);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidRotation")]
    public static extern byte IsValidRotation(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidVec2")]
    public static extern byte IsValidVec2(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetBodyA")]
    public static extern BodyId JointGetBodyA(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetBodyB")]
    public static extern BodyId JointGetBodyB(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetCollideConnected")]
    public static extern byte JointGetCollideConnected(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetConstraintForce")]
    public static extern Vec2 JointGetConstraintForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetConstraintTorque")]
    public static extern float JointGetConstraintTorque(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetLocalAnchorA")]
    public static extern Vec2 JointGetLocalAnchorA(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetLocalAnchorB")]
    public static extern Vec2 JointGetLocalAnchorB(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetType")]
    public static extern JointType JointGetType(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetUserData")]
    public static extern void* JointGetUserData(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetWorld")]
    public static extern WorldId JointGetWorld(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_IsValid")]
    public static extern byte JointIsValid(JointId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_SetCollideConnected")]
    public static extern void JointSetCollideConnected(JointId jointId, byte shouldCollide);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_SetUserData")]
    public static extern void JointSetUserData(JointId jointId, void* userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_WakeBodies")]
    public static extern void JointWakeBodies(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LeftPerp")]
    public static extern Vec2 LeftPerp(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Length")]
    public static extern float Length(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LengthSquared")]
    public static extern float LengthSquared(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Lerp")]
    public static extern Vec2 Lerp(Vec2 a, Vec2 b, float t);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LoadBodyId")]
    public static extern BodyId LoadBodyId(ulong x);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LoadChainId")]
    public static extern ChainId LoadChainId(ulong x);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LoadJointId")]
    public static extern JointId LoadJointId(ulong x);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LoadShapeId")]
    public static extern ShapeId LoadShapeId(ulong x);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeBox")]
    public static extern Polygon MakeBox(float halfWidth, float halfHeight);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetBox")]
    public static extern Polygon MakeOffsetBox(float halfWidth, float halfHeight, Vec2 center, Rot rotation);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetPolygon")]
    public static extern Polygon MakeOffsetPolygon(Hull* hull, Vec2 position, Rot rotation);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetRoundedBox")]
    public static extern Polygon MakeOffsetRoundedBox(float halfWidth, float halfHeight, Vec2 center, Rot rotation, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetRoundedPolygon")]
    public static extern Polygon MakeOffsetRoundedPolygon(Hull* hull, Vec2 position, Rot rotation, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakePolygon")]
    public static extern Polygon MakePolygon(Hull* hull, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeProxy")]
    public static extern ShapeProxy MakeProxy(Vec2* vertices, int count, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeRot")]
    public static extern Rot MakeRot(float radians);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeRoundedBox")]
    public static extern Polygon MakeRoundedBox(float halfWidth, float halfHeight, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeSquare")]
    public static extern Polygon MakeSquare(float halfWidth);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Max")]
    public static extern Vec2 Max(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MaxFloat")]
    public static extern float MaxFloat(float a, float b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MaxInt")]
    public static extern int MaxInt(int a, int b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Min")]
    public static extern Vec2 Min(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MinFloat")]
    public static extern float MinFloat(float a, float b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MinInt")]
    public static extern int MinInt(int a, int b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetAngularOffset")]
    public static extern float MotorJointGetAngularOffset(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetCorrectionFactor")]
    public static extern float MotorJointGetCorrectionFactor(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetLinearOffset")]
    public static extern Vec2 MotorJointGetLinearOffset(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetMaxForce")]
    public static extern float MotorJointGetMaxForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetMaxTorque")]
    public static extern float MotorJointGetMaxTorque(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetAngularOffset")]
    public static extern void MotorJointSetAngularOffset(JointId jointId, float angularOffset);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetCorrectionFactor")]
    public static extern void MotorJointSetCorrectionFactor(JointId jointId, float correctionFactor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetLinearOffset")]
    public static extern void MotorJointSetLinearOffset(JointId jointId, Vec2 linearOffset);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetMaxForce")]
    public static extern void MotorJointSetMaxForce(JointId jointId, float maxForce);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetMaxTorque")]
    public static extern void MotorJointSetMaxTorque(JointId jointId, float maxTorque);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetMaxForce")]
    public static extern float MouseJointGetMaxForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetSpringDampingRatio")]
    public static extern float MouseJointGetSpringDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetSpringHertz")]
    public static extern float MouseJointGetSpringHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetTarget")]
    public static extern Vec2 MouseJointGetTarget(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetMaxForce")]
    public static extern void MouseJointSetMaxForce(JointId jointId, float maxForce);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetSpringDampingRatio")]
    public static extern void MouseJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetSpringHertz")]
    public static extern void MouseJointSetSpringHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetTarget")]
    public static extern void MouseJointSetTarget(JointId jointId, Vec2 target);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Mul")]
    public static extern Vec2 Mul(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulAdd")]
    public static extern Vec2 MulAdd(Vec2 a, float s, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulMV")]
    public static extern Vec2 MulMV(Mat22 A, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulRot")]
    public static extern Rot MulRot(Rot q, Rot r);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulSub")]
    public static extern Vec2 MulSub(Vec2 a, float s, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulSV")]
    public static extern Vec2 MulSV(float s, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulTransforms")]
    public static extern Transform MulTransforms(Transform A, Transform B);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Neg")]
    public static extern Vec2 Neg(Vec2 a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NLerp")]
    public static extern Rot NLerp(Rot q1, Rot q2, float t);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Normalize")]
    public static extern Vec2 Normalize(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NormalizeRot")]
    public static extern Rot NormalizeRot(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInCapsule")]
    public static extern byte PointInCapsule(Vec2 point, Capsule* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInCircle")]
    public static extern byte PointInCircle(Vec2 point, Circle* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInPolygon")]
    public static extern byte PointInPolygon(Vec2 point, Polygon* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableLimit")]
    public static extern void PrismaticJointEnableLimit(JointId jointId, byte enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableMotor")]
    public static extern void PrismaticJointEnableMotor(JointId jointId, byte enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableSpring")]
    public static extern void PrismaticJointEnableSpring(JointId jointId, byte enableSpring);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetLowerLimit")]
    public static extern float PrismaticJointGetLowerLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMaxMotorForce")]
    public static extern float PrismaticJointGetMaxMotorForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorForce")]
    public static extern float PrismaticJointGetMotorForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorSpeed")]
    public static extern float PrismaticJointGetMotorSpeed(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpeed")]
    public static extern float PrismaticJointGetSpeed(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpringDampingRatio")]
    public static extern float PrismaticJointGetSpringDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpringHertz")]
    public static extern float PrismaticJointGetSpringHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetTranslation")]
    public static extern float PrismaticJointGetTranslation(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetUpperLimit")]
    public static extern float PrismaticJointGetUpperLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsLimitEnabled")]
    public static extern byte PrismaticJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsMotorEnabled")]
    public static extern byte PrismaticJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsSpringEnabled")]
    public static extern byte PrismaticJointIsSpringEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetLimits")]
    public static extern void PrismaticJointSetLimits(JointId jointId, float lower, float upper);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetMaxMotorForce")]
    public static extern void PrismaticJointSetMaxMotorForce(JointId jointId, float force);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetMotorSpeed")]
    public static extern void PrismaticJointSetMotorSpeed(JointId jointId, float motorSpeed);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetSpringDampingRatio")]
    public static extern void PrismaticJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetSpringHertz")]
    public static extern void PrismaticJointSetSpringHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastCapsule")]
    public static extern CastOutput RayCastCapsule(RayCastInput* input, Capsule* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastCircle")]
    public static extern CastOutput RayCastCircle(RayCastInput* input, Circle* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastPolygon")]
    public static extern CastOutput RayCastPolygon(RayCastInput* input, Polygon* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastSegment")]
    public static extern CastOutput RayCastSegment(RayCastInput* input, Segment* shape, byte oneSided);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RelativeAngle")]
    public static extern float RelativeAngle(Rot b, Rot a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableLimit")]
    public static extern void RevoluteJointEnableLimit(JointId jointId, byte enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableMotor")]
    public static extern void RevoluteJointEnableMotor(JointId jointId, byte enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableSpring")]
    public static extern void RevoluteJointEnableSpring(JointId jointId, byte enableSpring);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetAngle")]
    public static extern float RevoluteJointGetAngle(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetLowerLimit")]
    public static extern float RevoluteJointGetLowerLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMaxMotorTorque")]
    public static extern float RevoluteJointGetMaxMotorTorque(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMotorSpeed")]
    public static extern float RevoluteJointGetMotorSpeed(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMotorTorque")]
    public static extern float RevoluteJointGetMotorTorque(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetSpringDampingRatio")]
    public static extern float RevoluteJointGetSpringDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetSpringHertz")]
    public static extern float RevoluteJointGetSpringHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetUpperLimit")]
    public static extern float RevoluteJointGetUpperLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_IsLimitEnabled")]
    public static extern byte RevoluteJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_IsMotorEnabled")]
    public static extern byte RevoluteJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_IsSpringEnabled")]
    public static extern byte RevoluteJointIsSpringEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetLimits")]
    public static extern void RevoluteJointSetLimits(JointId jointId, float lower, float upper);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetMaxMotorTorque")]
    public static extern void RevoluteJointSetMaxMotorTorque(JointId jointId, float torque);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetMotorSpeed")]
    public static extern void RevoluteJointSetMotorSpeed(JointId jointId, float motorSpeed);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetSpringDampingRatio")]
    public static extern void RevoluteJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetSpringHertz")]
    public static extern void RevoluteJointSetSpringHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RightPerp")]
    public static extern Vec2 RightPerp(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_GetAngle")]
    public static extern float RotGetAngle(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_GetXAxis")]
    public static extern Vec2 RotGetXAxis(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_GetYAxis")]
    public static extern Vec2 RotGetYAxis(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RotateVector")]
    public static extern Vec2 RotateVector(Rot q, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SegmentDistance")]
    public static extern SegmentDistanceResult SegmentDistance(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetAllocator")]
    public static extern void SetAllocator(delegate* unmanaged<uint, int, void*> allocFcn, delegate* unmanaged<void*, void> freeFcn);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetAssertFcn")]
    public static extern void SetAssertFcn(delegate* unmanaged<byte*, byte*, int, int> assertFcn);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetLengthUnitsPerMeter")]
    public static extern void SetLengthUnitsPerMeter(float lengthUnits);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreContactEventsEnabled")]
    public static extern byte ShapeAreContactEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreHitEventsEnabled")]
    public static extern byte ShapeAreHitEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_ArePreSolveEventsEnabled")]
    public static extern byte ShapeArePreSolveEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableContactEvents")]
    public static extern void ShapeEnableContactEvents(ShapeId shapeId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableHitEvents")]
    public static extern void ShapeEnableHitEvents(ShapeId shapeId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnablePreSolveEvents")]
    public static extern void ShapeEnablePreSolveEvents(ShapeId shapeId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetAABB")]
    public static extern AABB ShapeGetAABB(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetBody")]
    public static extern BodyId ShapeGetBody(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetCapsule")]
    public static extern Capsule ShapeGetCapsule(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetChainSegment")]
    public static extern ChainSegment ShapeGetChainSegment(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetCircle")]
    public static extern Circle ShapeGetCircle(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetClosestPoint")]
    public static extern Vec2 ShapeGetClosestPoint(ShapeId shapeId, Vec2 target);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetContactCapacity")]
    public static extern int ShapeGetContactCapacity(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetContactData")]
    public static extern int ShapeGetContactData(ShapeId shapeId, ContactData* contactData, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetDensity")]
    public static extern float ShapeGetDensity(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetFilter")]
    public static extern Filter ShapeGetFilter(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetFriction")]
    public static extern float ShapeGetFriction(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetMassData")]
    public static extern MassData ShapeGetMassData(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetMaterial")]
    public static extern int ShapeGetMaterial(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetParentChain")]
    public static extern ChainId ShapeGetParentChain(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetPolygon")]
    public static extern Polygon ShapeGetPolygon(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetRestitution")]
    public static extern float ShapeGetRestitution(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSegment")]
    public static extern Segment ShapeGetSegment(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSensorCapacity")]
    public static extern int ShapeGetSensorCapacity(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSensorOverlaps")]
    public static extern int ShapeGetSensorOverlaps(ShapeId shapeId, ShapeId* overlaps, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetType")]
    public static extern ShapeType ShapeGetType(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetUserData")]
    public static extern void* ShapeGetUserData(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetWorld")]
    public static extern WorldId ShapeGetWorld(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_IsSensor")]
    public static extern byte ShapeIsSensor(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_IsValid")]
    public static extern byte ShapeIsValid(ShapeId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_RayCast")]
    public static extern CastOutput ShapeRayCast(ShapeId shapeId, RayCastInput* input);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetCapsule")]
    public static extern void ShapeSetCapsule(ShapeId shapeId, Capsule* capsule);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetCircle")]
    public static extern void ShapeSetCircle(ShapeId shapeId, Circle* circle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetDensity")]
    public static extern void ShapeSetDensity(ShapeId shapeId, float density, byte updateBodyMass);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetFilter")]
    public static extern void ShapeSetFilter(ShapeId shapeId, Filter filter);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetFriction")]
    public static extern void ShapeSetFriction(ShapeId shapeId, float friction);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetMaterial")]
    public static extern void ShapeSetMaterial(ShapeId shapeId, int material);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetPolygon")]
    public static extern void ShapeSetPolygon(ShapeId shapeId, Polygon* polygon);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetRestitution")]
    public static extern void ShapeSetRestitution(ShapeId shapeId, float restitution);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetSegment")]
    public static extern void ShapeSetSegment(ShapeId shapeId, Segment* segment);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetUserData")]
    public static extern void ShapeSetUserData(ShapeId shapeId, void* userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_TestPoint")]
    public static extern byte ShapeTestPoint(ShapeId shapeId, Vec2 point);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCast")]
    public static extern CastOutput ShapeCast(ShapeCastPairInput* input);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastCapsule")]
    public static extern CastOutput ShapeCastCapsule(ShapeCastInput* input, Capsule* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastCircle")]
    public static extern CastOutput ShapeCastCircle(ShapeCastInput* input, Circle* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastPolygon")]
    public static extern CastOutput ShapeCastPolygon(ShapeCastInput* input, Polygon* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastSegment")]
    public static extern CastOutput ShapeCastSegment(ShapeCastInput* input, Segment* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeDistance")]
    public static extern DistanceOutput ShapeDistance(SimplexCache* cache, DistanceInput* input, Simplex* simplexes, int simplexCapacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Solve22")]
    public static extern Vec2 Solve22(Mat22 A, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2StoreBodyId")]
    public static extern ulong StoreBodyId(BodyId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2StoreChainId")]
    public static extern ulong StoreChainId(ChainId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2StoreJointId")]
    public static extern ulong StoreJointId(JointId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2StoreShapeId")]
    public static extern ulong StoreShapeId(ShapeId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Sub")]
    public static extern Vec2 Sub(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TimeOfImpact")]
    public static extern TOIOutput TimeOfImpact(TOIInput* input);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPoint")]
    public static extern Vec2 TransformPoint(Transform t, Vec2 p);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPolygon")]
    public static extern Polygon TransformPolygon(Transform transform, Polygon* polygon);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2UnwindAngle")]
    public static extern float UnwindAngle(float radians);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2UnwindLargeAngle")]
    public static extern float UnwindLargeAngle(float radians);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ValidateHull")]
    public static extern byte ValidateHull(Hull* hull);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetAngularDampingRatio")]
    public static extern float WeldJointGetAngularDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetAngularHertz")]
    public static extern float WeldJointGetAngularHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetLinearDampingRatio")]
    public static extern float WeldJointGetLinearDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetLinearHertz")]
    public static extern float WeldJointGetLinearHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetReferenceAngle")]
    public static extern float WeldJointGetReferenceAngle(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetAngularDampingRatio")]
    public static extern void WeldJointSetAngularDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetAngularHertz")]
    public static extern void WeldJointSetAngularHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetLinearDampingRatio")]
    public static extern void WeldJointSetLinearDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetLinearHertz")]
    public static extern void WeldJointSetLinearHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetReferenceAngle")]
    public static extern void WeldJointSetReferenceAngle(JointId jointId, float angleInRadians);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableLimit")]
    public static extern void WheelJointEnableLimit(JointId jointId, byte enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableMotor")]
    public static extern void WheelJointEnableMotor(JointId jointId, byte enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableSpring")]
    public static extern void WheelJointEnableSpring(JointId jointId, byte enableSpring);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetLowerLimit")]
    public static extern float WheelJointGetLowerLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetMaxMotorTorque")]
    public static extern float WheelJointGetMaxMotorTorque(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetMotorSpeed")]
    public static extern float WheelJointGetMotorSpeed(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetMotorTorque")]
    public static extern float WheelJointGetMotorTorque(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetSpringDampingRatio")]
    public static extern float WheelJointGetSpringDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetSpringHertz")]
    public static extern float WheelJointGetSpringHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetUpperLimit")]
    public static extern float WheelJointGetUpperLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsLimitEnabled")]
    public static extern byte WheelJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsMotorEnabled")]
    public static extern byte WheelJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsSpringEnabled")]
    public static extern byte WheelJointIsSpringEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetLimits")]
    public static extern void WheelJointSetLimits(JointId jointId, float lower, float upper);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetMaxMotorTorque")]
    public static extern void WheelJointSetMaxMotorTorque(JointId jointId, float torque);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetMotorSpeed")]
    public static extern void WheelJointSetMotorSpeed(JointId jointId, float motorSpeed);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetSpringDampingRatio")]
    public static extern void WheelJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetSpringHertz")]
    public static extern void WheelJointSetSpringHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastCapsule")]
    public static extern TreeStats WorldCastCapsule(WorldId worldId, Capsule* capsule, Transform originTransform, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastCircle")]
    public static extern TreeStats WorldCastCircle(WorldId worldId, Circle* circle, Transform originTransform, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastPolygon")]
    public static extern TreeStats WorldCastPolygon(WorldId worldId, Polygon* polygon, Transform originTransform, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastRay")]
    public static extern TreeStats WorldCastRay(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastRayClosest")]
    public static extern RayResult WorldCastRayClosest(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Draw")]
    public static extern void WorldDraw(WorldId worldId, DebugDraw* draw);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_DumpMemoryStats")]
    public static extern void WorldDumpMemoryStats(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableContinuous")]
    public static extern void WorldEnableContinuous(WorldId worldId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableSleeping")]
    public static extern void WorldEnableSleeping(WorldId worldId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableSpeculative")]
    public static extern void WorldEnableSpeculative(WorldId worldId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableWarmStarting")]
    public static extern void WorldEnableWarmStarting(WorldId worldId, byte flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Explode")]
    public static extern void WorldExplode(WorldId worldId, ExplosionDef* explosionDef);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetAwakeBodyCount")]
    public static extern int WorldGetAwakeBodyCount(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetBodyEvents")]
    public static extern BodyEvents WorldGetBodyEvents(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetContactEvents")]
    public static extern ContactEvents WorldGetContactEvents(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetCounters")]
    public static extern Counters WorldGetCounters(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetGravity")]
    public static extern Vec2 WorldGetGravity(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetHitEventThreshold")]
    public static extern float WorldGetHitEventThreshold(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetMaximumLinearSpeed")]
    public static extern float WorldGetMaximumLinearSpeed(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetProfile")]
    public static extern Profile WorldGetProfile(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetRestitutionThreshold")]
    public static extern float WorldGetRestitutionThreshold(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetSensorEvents")]
    public static extern SensorEvents WorldGetSensorEvents(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetUserData")]
    public static extern void* WorldGetUserData(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsContinuousEnabled")]
    public static extern byte WorldIsContinuousEnabled(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsSleepingEnabled")]
    public static extern byte WorldIsSleepingEnabled(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsValid")]
    public static extern byte WorldIsValid(WorldId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsWarmStartingEnabled")]
    public static extern byte WorldIsWarmStartingEnabled(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapAABB")]
    public static extern TreeStats WorldOverlapAABB(WorldId worldId, AABB aabb, QueryFilter filter, delegate* unmanaged<ShapeId, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCapsule")]
    public static extern TreeStats WorldOverlapCapsule(WorldId worldId, Capsule* capsule, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCircle")]
    public static extern TreeStats WorldOverlapCircle(WorldId worldId, Circle* circle, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapPoint")]
    public static extern TreeStats WorldOverlapPoint(WorldId worldId, Vec2 point, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapPolygon")]
    public static extern TreeStats WorldOverlapPolygon(WorldId worldId, Polygon* polygon, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_RebuildStaticTree")]
    public static extern void WorldRebuildStaticTree(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetContactTuning")]
    public static extern void WorldSetContactTuning(WorldId worldId, float hertz, float dampingRatio, float pushSpeed);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetCustomFilterCallback")]
    public static extern void WorldSetCustomFilterCallback(WorldId worldId, delegate* unmanaged<ShapeId, ShapeId, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetFrictionCallback")]
    public static extern void WorldSetFrictionCallback(WorldId worldId, delegate* unmanaged<float, int, float, int, float> callback);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetGravity")]
    public static extern void WorldSetGravity(WorldId worldId, Vec2 gravity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetHitEventThreshold")]
    public static extern void WorldSetHitEventThreshold(WorldId worldId, float value);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetJointTuning")]
    public static extern void WorldSetJointTuning(WorldId worldId, float hertz, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetMaximumLinearSpeed")]
    public static extern void WorldSetMaximumLinearSpeed(WorldId worldId, float maximumLinearSpeed);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetPreSolveCallback")]
    public static extern void WorldSetPreSolveCallback(WorldId worldId, delegate* unmanaged<ShapeId, ShapeId, Manifold*, void*, byte> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetRestitutionCallback")]
    public static extern void WorldSetRestitutionCallback(WorldId worldId, delegate* unmanaged<float, int, float, int, float> callback);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetRestitutionThreshold")]
    public static extern void WorldSetRestitutionThreshold(WorldId worldId, float value);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetUserData")]
    public static extern void WorldSetUserData(WorldId worldId, void* userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Step")]
    public static extern void WorldStep(WorldId worldId, float timeStep, int subStepCount);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Yield")]
    public static extern void Yield();

    public enum BodyType : uint
    {
        staticBody = 0,
        kinematicBody = 1,
        dynamicBody = 2,
        bodyTypeCount = 3
    }

    public enum HexColor : uint
    {
        colorAliceBlue = 15792383,
        colorAntiqueWhite = 16444375,
        colorAqua = 65535,
        colorAquamarine = 8388564,
        colorAzure = 15794175,
        colorBeige = 16119260,
        colorBisque = 16770244,
        colorBlack = 0,
        colorBlanchedAlmond = 16772045,
        colorBlue = 255,
        colorBlueViolet = 9055202,
        colorBrown = 10824234,
        colorBurlywood = 14596231,
        colorCadetBlue = 6266528,
        colorChartreuse = 8388352,
        colorChocolate = 13789470,
        colorCoral = 16744272,
        colorCornflowerBlue = 6591981,
        colorCornsilk = 16775388,
        colorCrimson = 14423100,
        colorCyan = 65535,
        colorDarkBlue = 139,
        colorDarkCyan = 35723,
        colorDarkGoldenRod = 12092939,
        colorDarkGray = 11119017,
        colorDarkGreen = 25600,
        colorDarkKhaki = 12433259,
        colorDarkMagenta = 9109643,
        colorDarkOliveGreen = 5597999,
        colorDarkOrange = 16747520,
        colorDarkOrchid = 10040012,
        colorDarkRed = 9109504,
        colorDarkSalmon = 15308410,
        colorDarkSeaGreen = 9419919,
        colorDarkSlateBlue = 4734347,
        colorDarkSlateGray = 3100495,
        colorDarkTurquoise = 52945,
        colorDarkViolet = 9699539,
        colorDeepPink = 16716947,
        colorDeepSkyBlue = 49151,
        colorDimGray = 6908265,
        colorDodgerBlue = 2003199,
        colorFireBrick = 11674146,
        colorFloralWhite = 16775920,
        colorForestGreen = 2263842,
        colorFuchsia = 16711935,
        colorGainsboro = 14474460,
        colorGhostWhite = 16316671,
        colorGold = 16766720,
        colorGoldenRod = 14329120,
        colorGray = 8421504,
        colorGreen = 32768,
        colorGreenYellow = 11403055,
        colorHoneyDew = 15794160,
        colorHotPink = 16738740,
        colorIndianRed = 13458524,
        colorIndigo = 4915330,
        colorIvory = 16777200,
        colorKhaki = 15787660,
        colorLavender = 15132410,
        colorLavenderBlush = 16773365,
        colorLawnGreen = 8190976,
        colorLemonChiffon = 16775885,
        colorLightBlue = 11393254,
        colorLightCoral = 15761536,
        colorLightCyan = 14745599,
        colorLightGoldenRodYellow = 16448210,
        colorLightGray = 13882323,
        colorLightGreen = 9498256,
        colorLightPink = 16758465,
        colorLightSalmon = 16752762,
        colorLightSeaGreen = 2142890,
        colorLightSkyBlue = 8900346,
        colorLightSlateGray = 7833753,
        colorLightSteelBlue = 11584734,
        colorLightYellow = 16777184,
        colorLime = 65280,
        colorLimeGreen = 3329330,
        colorLinen = 16445670,
        colorMagenta = 16711935,
        colorMaroon = 8388608,
        colorMediumAquaMarine = 6737322,
        colorMediumBlue = 205,
        colorMediumOrchid = 12211667,
        colorMediumPurple = 9662683,
        colorMediumSeaGreen = 3978097,
        colorMediumSlateBlue = 8087790,
        colorMediumSpringGreen = 64154,
        colorMediumTurquoise = 4772300,
        colorMediumVioletRed = 13047173,
        colorMidnightBlue = 1644912,
        colorMintCream = 16121850,
        colorMistyRose = 16770273,
        colorMoccasin = 16770229,
        colorNavajoWhite = 16768685,
        colorNavy = 128,
        colorOldLace = 16643558,
        colorOlive = 8421376,
        colorOliveDrab = 7048739,
        colorOrange = 16753920,
        colorOrangeRed = 16729344,
        colorOrchid = 14315734,
        colorPaleGoldenRod = 15657130,
        colorPaleGreen = 10025880,
        colorPaleTurquoise = 11529966,
        colorPaleVioletRed = 14381203,
        colorPapayaWhip = 16773077,
        colorPeachPuff = 16767673,
        colorPeru = 13468991,
        colorPink = 16761035,
        colorPlum = 14524637,
        colorPowderBlue = 11591910,
        colorPurple = 8388736,
        colorRebeccaPurple = 6697881,
        colorRed = 16711680,
        colorRosyBrown = 12357519,
        colorRoyalBlue = 4286945,
        colorSaddleBrown = 9127187,
        colorSalmon = 16416882,
        colorSandyBrown = 16032864,
        colorSeaGreen = 3050327,
        colorSeaShell = 16774638,
        colorSienna = 10506797,
        colorSilver = 12632256,
        colorSkyBlue = 8900331,
        colorSlateBlue = 6970061,
        colorSlateGray = 7372944,
        colorSnow = 16775930,
        colorSpringGreen = 65407,
        colorSteelBlue = 4620980,
        colorTan = 13808780,
        colorTeal = 32896,
        colorThistle = 14204888,
        colorTomato = 16737095,
        colorTurquoise = 4251856,
        colorViolet = 15631086,
        colorWheat = 16113331,
        colorWhite = 16777215,
        colorWhiteSmoke = 16119285,
        colorYellow = 16776960,
        colorYellowGreen = 10145074,
        colorBox2DRed = 14430514,
        colorBox2DBlue = 3190463,
        colorBox2DGreen = 9226532,
        colorBox2DYellow = 16772748
    }

    public enum JointType : uint
    {
        distanceJoint = 0,
        motorJoint = 1,
        mouseJoint = 2,
        nullJoint = 3,
        prismaticJoint = 4,
        revoluteJoint = 5,
        weldJoint = 6,
        wheelJoint = 7
    }

    public enum ShapeType : uint
    {
        circleShape = 0,
        capsuleShape = 1,
        segmentShape = 2,
        polygonShape = 3,
        chainSegmentShape = 4,
        shapeTypeCount = 5
    }

    public enum TOIState : uint
    {
        toiStateUnknown = 0,
        toiStateFailed = 1,
        toiStateOverlapped = 2,
        toiStateHit = 3,
        toiStateSeparated = 4
    }

    public const BodyType staticBody = BodyType.staticBody;

    public const BodyType kinematicBody = BodyType.kinematicBody;

    public const BodyType dynamicBody = BodyType.dynamicBody;

    public const BodyType bodyTypeCount = BodyType.bodyTypeCount;

    public const HexColor colorAliceBlue = HexColor.colorAliceBlue;

    public const HexColor colorAntiqueWhite = HexColor.colorAntiqueWhite;

    public const HexColor colorAqua = HexColor.colorAqua;

    public const HexColor colorAquamarine = HexColor.colorAquamarine;

    public const HexColor colorAzure = HexColor.colorAzure;

    public const HexColor colorBeige = HexColor.colorBeige;

    public const HexColor colorBisque = HexColor.colorBisque;

    public const HexColor colorBlack = HexColor.colorBlack;

    public const HexColor colorBlanchedAlmond = HexColor.colorBlanchedAlmond;

    public const HexColor colorBlue = HexColor.colorBlue;

    public const HexColor colorBlueViolet = HexColor.colorBlueViolet;

    public const HexColor colorBrown = HexColor.colorBrown;

    public const HexColor colorBurlywood = HexColor.colorBurlywood;

    public const HexColor colorCadetBlue = HexColor.colorCadetBlue;

    public const HexColor colorChartreuse = HexColor.colorChartreuse;

    public const HexColor colorChocolate = HexColor.colorChocolate;

    public const HexColor colorCoral = HexColor.colorCoral;

    public const HexColor colorCornflowerBlue = HexColor.colorCornflowerBlue;

    public const HexColor colorCornsilk = HexColor.colorCornsilk;

    public const HexColor colorCrimson = HexColor.colorCrimson;

    public const HexColor colorCyan = HexColor.colorCyan;

    public const HexColor colorDarkBlue = HexColor.colorDarkBlue;

    public const HexColor colorDarkCyan = HexColor.colorDarkCyan;

    public const HexColor colorDarkGoldenRod = HexColor.colorDarkGoldenRod;

    public const HexColor colorDarkGray = HexColor.colorDarkGray;

    public const HexColor colorDarkGreen = HexColor.colorDarkGreen;

    public const HexColor colorDarkKhaki = HexColor.colorDarkKhaki;

    public const HexColor colorDarkMagenta = HexColor.colorDarkMagenta;

    public const HexColor colorDarkOliveGreen = HexColor.colorDarkOliveGreen;

    public const HexColor colorDarkOrange = HexColor.colorDarkOrange;

    public const HexColor colorDarkOrchid = HexColor.colorDarkOrchid;

    public const HexColor colorDarkRed = HexColor.colorDarkRed;

    public const HexColor colorDarkSalmon = HexColor.colorDarkSalmon;

    public const HexColor colorDarkSeaGreen = HexColor.colorDarkSeaGreen;

    public const HexColor colorDarkSlateBlue = HexColor.colorDarkSlateBlue;

    public const HexColor colorDarkSlateGray = HexColor.colorDarkSlateGray;

    public const HexColor colorDarkTurquoise = HexColor.colorDarkTurquoise;

    public const HexColor colorDarkViolet = HexColor.colorDarkViolet;

    public const HexColor colorDeepPink = HexColor.colorDeepPink;

    public const HexColor colorDeepSkyBlue = HexColor.colorDeepSkyBlue;

    public const HexColor colorDimGray = HexColor.colorDimGray;

    public const HexColor colorDodgerBlue = HexColor.colorDodgerBlue;

    public const HexColor colorFireBrick = HexColor.colorFireBrick;

    public const HexColor colorFloralWhite = HexColor.colorFloralWhite;

    public const HexColor colorForestGreen = HexColor.colorForestGreen;

    public const HexColor colorFuchsia = HexColor.colorFuchsia;

    public const HexColor colorGainsboro = HexColor.colorGainsboro;

    public const HexColor colorGhostWhite = HexColor.colorGhostWhite;

    public const HexColor colorGold = HexColor.colorGold;

    public const HexColor colorGoldenRod = HexColor.colorGoldenRod;

    public const HexColor colorGray = HexColor.colorGray;

    public const HexColor colorGreen = HexColor.colorGreen;

    public const HexColor colorGreenYellow = HexColor.colorGreenYellow;

    public const HexColor colorHoneyDew = HexColor.colorHoneyDew;

    public const HexColor colorHotPink = HexColor.colorHotPink;

    public const HexColor colorIndianRed = HexColor.colorIndianRed;

    public const HexColor colorIndigo = HexColor.colorIndigo;

    public const HexColor colorIvory = HexColor.colorIvory;

    public const HexColor colorKhaki = HexColor.colorKhaki;

    public const HexColor colorLavender = HexColor.colorLavender;

    public const HexColor colorLavenderBlush = HexColor.colorLavenderBlush;

    public const HexColor colorLawnGreen = HexColor.colorLawnGreen;

    public const HexColor colorLemonChiffon = HexColor.colorLemonChiffon;

    public const HexColor colorLightBlue = HexColor.colorLightBlue;

    public const HexColor colorLightCoral = HexColor.colorLightCoral;

    public const HexColor colorLightCyan = HexColor.colorLightCyan;

    public const HexColor colorLightGoldenRodYellow = HexColor.colorLightGoldenRodYellow;

    public const HexColor colorLightGray = HexColor.colorLightGray;

    public const HexColor colorLightGreen = HexColor.colorLightGreen;

    public const HexColor colorLightPink = HexColor.colorLightPink;

    public const HexColor colorLightSalmon = HexColor.colorLightSalmon;

    public const HexColor colorLightSeaGreen = HexColor.colorLightSeaGreen;

    public const HexColor colorLightSkyBlue = HexColor.colorLightSkyBlue;

    public const HexColor colorLightSlateGray = HexColor.colorLightSlateGray;

    public const HexColor colorLightSteelBlue = HexColor.colorLightSteelBlue;

    public const HexColor colorLightYellow = HexColor.colorLightYellow;

    public const HexColor colorLime = HexColor.colorLime;

    public const HexColor colorLimeGreen = HexColor.colorLimeGreen;

    public const HexColor colorLinen = HexColor.colorLinen;

    public const HexColor colorMagenta = HexColor.colorMagenta;

    public const HexColor colorMaroon = HexColor.colorMaroon;

    public const HexColor colorMediumAquaMarine = HexColor.colorMediumAquaMarine;

    public const HexColor colorMediumBlue = HexColor.colorMediumBlue;

    public const HexColor colorMediumOrchid = HexColor.colorMediumOrchid;

    public const HexColor colorMediumPurple = HexColor.colorMediumPurple;

    public const HexColor colorMediumSeaGreen = HexColor.colorMediumSeaGreen;

    public const HexColor colorMediumSlateBlue = HexColor.colorMediumSlateBlue;

    public const HexColor colorMediumSpringGreen = HexColor.colorMediumSpringGreen;

    public const HexColor colorMediumTurquoise = HexColor.colorMediumTurquoise;

    public const HexColor colorMediumVioletRed = HexColor.colorMediumVioletRed;

    public const HexColor colorMidnightBlue = HexColor.colorMidnightBlue;

    public const HexColor colorMintCream = HexColor.colorMintCream;

    public const HexColor colorMistyRose = HexColor.colorMistyRose;

    public const HexColor colorMoccasin = HexColor.colorMoccasin;

    public const HexColor colorNavajoWhite = HexColor.colorNavajoWhite;

    public const HexColor colorNavy = HexColor.colorNavy;

    public const HexColor colorOldLace = HexColor.colorOldLace;

    public const HexColor colorOlive = HexColor.colorOlive;

    public const HexColor colorOliveDrab = HexColor.colorOliveDrab;

    public const HexColor colorOrange = HexColor.colorOrange;

    public const HexColor colorOrangeRed = HexColor.colorOrangeRed;

    public const HexColor colorOrchid = HexColor.colorOrchid;

    public const HexColor colorPaleGoldenRod = HexColor.colorPaleGoldenRod;

    public const HexColor colorPaleGreen = HexColor.colorPaleGreen;

    public const HexColor colorPaleTurquoise = HexColor.colorPaleTurquoise;

    public const HexColor colorPaleVioletRed = HexColor.colorPaleVioletRed;

    public const HexColor colorPapayaWhip = HexColor.colorPapayaWhip;

    public const HexColor colorPeachPuff = HexColor.colorPeachPuff;

    public const HexColor colorPeru = HexColor.colorPeru;

    public const HexColor colorPink = HexColor.colorPink;

    public const HexColor colorPlum = HexColor.colorPlum;

    public const HexColor colorPowderBlue = HexColor.colorPowderBlue;

    public const HexColor colorPurple = HexColor.colorPurple;

    public const HexColor colorRebeccaPurple = HexColor.colorRebeccaPurple;

    public const HexColor colorRed = HexColor.colorRed;

    public const HexColor colorRosyBrown = HexColor.colorRosyBrown;

    public const HexColor colorRoyalBlue = HexColor.colorRoyalBlue;

    public const HexColor colorSaddleBrown = HexColor.colorSaddleBrown;

    public const HexColor colorSalmon = HexColor.colorSalmon;

    public const HexColor colorSandyBrown = HexColor.colorSandyBrown;

    public const HexColor colorSeaGreen = HexColor.colorSeaGreen;

    public const HexColor colorSeaShell = HexColor.colorSeaShell;

    public const HexColor colorSienna = HexColor.colorSienna;

    public const HexColor colorSilver = HexColor.colorSilver;

    public const HexColor colorSkyBlue = HexColor.colorSkyBlue;

    public const HexColor colorSlateBlue = HexColor.colorSlateBlue;

    public const HexColor colorSlateGray = HexColor.colorSlateGray;

    public const HexColor colorSnow = HexColor.colorSnow;

    public const HexColor colorSpringGreen = HexColor.colorSpringGreen;

    public const HexColor colorSteelBlue = HexColor.colorSteelBlue;

    public const HexColor colorTan = HexColor.colorTan;

    public const HexColor colorTeal = HexColor.colorTeal;

    public const HexColor colorThistle = HexColor.colorThistle;

    public const HexColor colorTomato = HexColor.colorTomato;

    public const HexColor colorTurquoise = HexColor.colorTurquoise;

    public const HexColor colorViolet = HexColor.colorViolet;

    public const HexColor colorWheat = HexColor.colorWheat;

    public const HexColor colorWhite = HexColor.colorWhite;

    public const HexColor colorWhiteSmoke = HexColor.colorWhiteSmoke;

    public const HexColor colorYellow = HexColor.colorYellow;

    public const HexColor colorYellowGreen = HexColor.colorYellowGreen;

    public const HexColor colorBox2DRed = HexColor.colorBox2DRed;

    public const HexColor colorBox2DBlue = HexColor.colorBox2DBlue;

    public const HexColor colorBox2DGreen = HexColor.colorBox2DGreen;

    public const HexColor colorBox2DYellow = HexColor.colorBox2DYellow;

    public const JointType distanceJoint = JointType.distanceJoint;

    public const JointType motorJoint = JointType.motorJoint;

    public const JointType mouseJoint = JointType.mouseJoint;

    public const JointType nullJoint = JointType.nullJoint;

    public const JointType prismaticJoint = JointType.prismaticJoint;

    public const JointType revoluteJoint = JointType.revoluteJoint;

    public const JointType weldJoint = JointType.weldJoint;

    public const JointType wheelJoint = JointType.wheelJoint;

    public const ShapeType circleShape = ShapeType.circleShape;

    public const ShapeType capsuleShape = ShapeType.capsuleShape;

    public const ShapeType segmentShape = ShapeType.segmentShape;

    public const ShapeType polygonShape = ShapeType.polygonShape;

    public const ShapeType chainSegmentShape = ShapeType.chainSegmentShape;

    public const ShapeType shapeTypeCount = ShapeType.shapeTypeCount;

    public const TOIState toiStateUnknown = TOIState.toiStateUnknown;

    public const TOIState toiStateFailed = TOIState.toiStateFailed;

    public const TOIState toiStateOverlapped = TOIState.toiStateOverlapped;

    public const TOIState toiStateHit = TOIState.toiStateHit;

    public const TOIState toiStateSeparated = TOIState.toiStateSeparated;

    public partial struct Version
    {
        public int major;

        public int minor;

        public int revision;
    }

    public partial struct Vec2
    {
        public float x;

        public float y;
    }

    public partial struct CosSin
    {
        public float cosine;

        public float sine;
    }

    public partial struct Rot
    {
        public float c;

        public float s;
    }

    public partial struct Transform
    {
        public Vec2 p;

        public Rot q;
    }

    public partial struct Mat22
    {
        public Vec2 cx;

        public Vec2 cy;
    }

    public partial struct AABB
    {
        public Vec2 lowerBound;

        public Vec2 upperBound;
    }

    public partial struct SimplexCache
    {
        public ushort count;

        public InlineArrays.byte_3 indexA;

        public InlineArrays.byte_3 indexB;
    }

    public partial struct Hull
    {
        public InlineArrays.Vec2_8 points;

        public int count;
    }

    public partial struct RayCastInput
    {
        public Vec2 origin;

        public Vec2 translation;

        public float maxFraction;
    }

    public partial struct ShapeCastInput
    {
        public InlineArrays.Vec2_8 points;

        public int count;

        public float radius;

        public Vec2 translation;

        public float maxFraction;
    }

    public partial struct CastOutput
    {
        public Vec2 normal;

        public Vec2 point;

        public float fraction;

        public int iterations;

        public byte hit;
    }

    public partial struct MassData
    {
        public float mass;

        public Vec2 center;

        public float rotationalInertia;
    }

    public partial struct Circle
    {
        public Vec2 center;

        public float radius;
    }

    public partial struct Capsule
    {
        public Vec2 center1;

        public Vec2 center2;

        public float radius;
    }

    public partial struct Polygon
    {
        public InlineArrays.Vec2_8 vertices;

        public InlineArrays.Vec2_8 normals;

        public Vec2 centroid;

        public float radius;

        public int count;
    }

    public partial struct Segment
    {
        public Vec2 point1;

        public Vec2 point2;
    }

    public partial struct ChainSegment
    {
        public Vec2 ghost1;

        public Segment segment;

        public Vec2 ghost2;

        public int chainId;
    }

    public partial struct SegmentDistanceResult
    {
        public Vec2 closest1;

        public Vec2 closest2;

        public float fraction1;

        public float fraction2;

        public float distanceSquared;
    }

    public partial struct ShapeProxy
    {
        public InlineArrays.Vec2_8 points;

        public int count;

        public float radius;
    }

    public partial struct DistanceInput
    {
        public ShapeProxy proxyA;

        public ShapeProxy proxyB;

        public Transform transformA;

        public Transform transformB;

        public byte useRadii;
    }

    public partial struct DistanceOutput
    {
        public Vec2 pointA;

        public Vec2 pointB;

        public float distance;

        public int iterations;

        public int simplexCount;
    }

    public partial struct SimplexVertex
    {
        public Vec2 wA;

        public Vec2 wB;

        public Vec2 w;

        public float a;

        public int indexA;

        public int indexB;
    }

    public partial struct Simplex
    {
        public SimplexVertex v1;

        public SimplexVertex v2;

        public SimplexVertex v3;

        public int count;
    }

    public partial struct ShapeCastPairInput
    {
        public ShapeProxy proxyA;

        public ShapeProxy proxyB;

        public Transform transformA;

        public Transform transformB;

        public Vec2 translationB;

        public float maxFraction;
    }

    public partial struct Sweep
    {
        public Vec2 localCenter;

        public Vec2 c1;

        public Vec2 c2;

        public Rot q1;

        public Rot q2;
    }

    public partial struct TOIInput
    {
        public ShapeProxy proxyA;

        public ShapeProxy proxyB;

        public Sweep sweepA;

        public Sweep sweepB;

        public float maxFraction;
    }

    public partial struct TOIOutput
    {
        public TOIState state;

        public float fraction;
    }

    public partial struct ManifoldPoint
    {
        public Vec2 point;

        public Vec2 anchorA;

        public Vec2 anchorB;

        public float separation;

        public float normalImpulse;

        public float tangentImpulse;

        public float maxNormalImpulse;

        public float normalVelocity;

        public ushort id;

        public byte persisted;
    }

    public partial struct Manifold
    {
        public Vec2 normal;

        public float rollingImpulse;

        public InlineArrays.ManifoldPoint_2 points;

        public int pointCount;
    }

    public partial struct DynamicTree
    {
        public TreeNode* nodes;

        public int root;

        public int nodeCount;

        public int nodeCapacity;

        public int freeList;

        public int proxyCount;

        public int* leafIndices;

        public AABB* leafBoxes;

        public Vec2* leafCenters;

        public int* binIndices;

        public int rebuildCapacity;
    }

    public partial struct TreeNode
    {
    }

    public partial struct TreeStats
    {
        public int nodeVisits;

        public int leafVisits;
    }

    public partial struct WorldId
    {
        public ushort index1;

        public ushort generation;
    }

    public partial struct BodyId
    {
        public int index1;

        public ushort world0;

        public ushort generation;
    }

    public partial struct ShapeId
    {
        public int index1;

        public ushort world0;

        public ushort generation;
    }

    public partial struct ChainId
    {
        public int index1;

        public ushort world0;

        public ushort generation;
    }

    public partial struct JointId
    {
        public int index1;

        public ushort world0;

        public ushort generation;
    }

    public partial struct RayResult
    {
        public ShapeId shapeId;

        public Vec2 point;

        public Vec2 normal;

        public float fraction;

        public int nodeVisits;

        public int leafVisits;

        public byte hit;
    }

    public partial struct WorldDef
    {
        public Vec2 gravity;

        public float restitutionThreshold;

        public float hitEventThreshold;

        public float contactHertz;

        public float contactDampingRatio;

        public float contactPushMaxSpeed;

        public float jointHertz;

        public float jointDampingRatio;

        public float maximumLinearSpeed;

        public delegate* unmanaged<float, int, float, int, float> frictionCallback;

        public delegate* unmanaged<float, int, float, int, float> restitutionCallback;

        public byte enableSleep;

        public byte enableContinuous;

        public int workerCount;

        public delegate* unmanaged<delegate* unmanaged<int, int, uint, void*, void> , int, int, void*, void*, void*> enqueueTask;

        public delegate* unmanaged<void*, void*, void> finishTask;

        public void* userTaskContext;

        public void* userData;

        public int internalValue;
    }

    public partial struct BodyDef
    {
        public BodyType type;

        public Vec2 position;

        public Rot rotation;

        public Vec2 linearVelocity;

        public float angularVelocity;

        public float linearDamping;

        public float angularDamping;

        public float gravityScale;

        public float sleepThreshold;

        public byte* name;

        public void* userData;

        public byte enableSleep;

        public byte isAwake;

        public byte fixedRotation;

        public byte isBullet;

        public byte isEnabled;

        public byte allowFastRotation;

        public int internalValue;
    }

    public partial struct Filter
    {
        public ulong categoryBits;

        public ulong maskBits;

        public int groupIndex;
    }

    public partial struct QueryFilter
    {
        public ulong categoryBits;

        public ulong maskBits;
    }

    public partial struct ShapeDef
    {
        public void* userData;

        public float friction;

        public float restitution;

        public float rollingResistance;

        public float tangentSpeed;

        public int material;

        public float density;

        public Filter filter;

        public uint customColor;

        public byte isSensor;

        public byte enableContactEvents;

        public byte enableHitEvents;

        public byte enablePreSolveEvents;

        public byte invokeContactCreation;

        public byte updateBodyMass;

        public int internalValue;
    }

    public partial struct SurfaceMaterial
    {
        public float friction;

        public float restitution;

        public float rollingResistance;

        public float tangentSpeed;

        public int material;

        public uint customColor;
    }

    public partial struct ChainDef
    {
        public void* userData;

        public Vec2* points;

        public int count;

        public SurfaceMaterial* materials;

        public int materialCount;

        public Filter filter;

        public byte isLoop;

        public int internalValue;
    }

    public partial struct Profile
    {
        public float step;

        public float pairs;

        public float collide;

        public float solve;

        public float mergeIslands;

        public float prepareStages;

        public float solveConstraints;

        public float prepareConstraints;

        public float integrateVelocities;

        public float warmStart;

        public float solveImpulses;

        public float integratePositions;

        public float relaxImpulses;

        public float applyRestitution;

        public float storeImpulses;

        public float splitIslands;

        public float transforms;

        public float hitEvents;

        public float refit;

        public float bullets;

        public float sleepIslands;

        public float sensors;
    }

    public partial struct Counters
    {
        public int bodyCount;

        public int shapeCount;

        public int contactCount;

        public int jointCount;

        public int islandCount;

        public int stackUsed;

        public int staticTreeHeight;

        public int treeHeight;

        public int byteCount;

        public int taskCount;

        public InlineArrays.int_12 colorCounts;
    }

    public partial struct DistanceJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 localAnchorA;

        public Vec2 localAnchorB;

        public float length;

        public byte enableSpring;

        public float hertz;

        public float dampingRatio;

        public byte enableLimit;

        public float minLength;

        public float maxLength;

        public byte enableMotor;

        public float maxMotorForce;

        public float motorSpeed;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct MotorJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 linearOffset;

        public float angularOffset;

        public float maxForce;

        public float maxTorque;

        public float correctionFactor;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct MouseJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 target;

        public float hertz;

        public float dampingRatio;

        public float maxForce;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct NullJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public void* userData;

        public int internalValue;
    }

    public partial struct PrismaticJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 localAnchorA;

        public Vec2 localAnchorB;

        public Vec2 localAxisA;

        public float referenceAngle;

        public byte enableSpring;

        public float hertz;

        public float dampingRatio;

        public byte enableLimit;

        public float lowerTranslation;

        public float upperTranslation;

        public byte enableMotor;

        public float maxMotorForce;

        public float motorSpeed;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct RevoluteJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 localAnchorA;

        public Vec2 localAnchorB;

        public float referenceAngle;

        public byte enableSpring;

        public float hertz;

        public float dampingRatio;

        public byte enableLimit;

        public float lowerAngle;

        public float upperAngle;

        public byte enableMotor;

        public float maxMotorTorque;

        public float motorSpeed;

        public float drawSize;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct WeldJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 localAnchorA;

        public Vec2 localAnchorB;

        public float referenceAngle;

        public float linearHertz;

        public float angularHertz;

        public float linearDampingRatio;

        public float angularDampingRatio;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct WheelJointDef
    {
        public BodyId bodyIdA;

        public BodyId bodyIdB;

        public Vec2 localAnchorA;

        public Vec2 localAnchorB;

        public Vec2 localAxisA;

        public byte enableSpring;

        public float hertz;

        public float dampingRatio;

        public byte enableLimit;

        public float lowerTranslation;

        public float upperTranslation;

        public byte enableMotor;

        public float maxMotorTorque;

        public float motorSpeed;

        public byte collideConnected;

        public void* userData;

        public int internalValue;
    }

    public partial struct ExplosionDef
    {
        public ulong maskBits;

        public Vec2 position;

        public float radius;

        public float falloff;

        public float impulsePerLength;
    }

    public partial struct SensorBeginTouchEvent
    {
        public ShapeId sensorShapeId;

        public ShapeId visitorShapeId;
    }

    public partial struct SensorEndTouchEvent
    {
        public ShapeId sensorShapeId;

        public ShapeId visitorShapeId;
    }

    public partial struct SensorEvents
    {
        public SensorBeginTouchEvent* beginEvents;

        public SensorEndTouchEvent* endEvents;

        public int beginCount;

        public int endCount;
    }

    public partial struct ContactBeginTouchEvent
    {
        public ShapeId shapeIdA;

        public ShapeId shapeIdB;

        public Manifold manifold;
    }

    public partial struct ContactEndTouchEvent
    {
        public ShapeId shapeIdA;

        public ShapeId shapeIdB;
    }

    public partial struct ContactHitEvent
    {
        public ShapeId shapeIdA;

        public ShapeId shapeIdB;

        public Vec2 point;

        public Vec2 normal;

        public float approachSpeed;
    }

    public partial struct ContactEvents
    {
        public ContactBeginTouchEvent* beginEvents;

        public ContactEndTouchEvent* endEvents;

        public ContactHitEvent* hitEvents;

        public int beginCount;

        public int endCount;

        public int hitCount;
    }

    public partial struct BodyMoveEvent
    {
        public Transform transform;

        public BodyId bodyId;

        public void* userData;

        public byte fellAsleep;
    }

    public partial struct BodyEvents
    {
        public BodyMoveEvent* moveEvents;

        public int moveCount;
    }

    public partial struct ContactData
    {
        public ShapeId shapeIdA;

        public ShapeId shapeIdB;

        public Manifold manifold;
    }

    public partial struct DebugDraw
    {
        public delegate* unmanaged<Vec2*, int, HexColor, void*, void> DrawPolygon;

        public delegate* unmanaged<Transform, Vec2*, int, float, HexColor, void*, void> DrawSolidPolygon;

        public delegate* unmanaged<Vec2, float, HexColor, void*, void> DrawCircle;

        public delegate* unmanaged<Transform, float, HexColor, void*, void> DrawSolidCircle;

        public delegate* unmanaged<Vec2, Vec2, float, HexColor, void*, void> DrawSolidCapsule;

        public delegate* unmanaged<Vec2, Vec2, HexColor, void*, void> DrawSegment;

        public delegate* unmanaged<Transform, void*, void> DrawTransform;

        public delegate* unmanaged<Vec2, float, HexColor, void*, void> DrawPoint;

        public delegate* unmanaged<Vec2, byte*, HexColor, void*, void> DrawString;

        public AABB drawingBounds;

        public byte useDrawingBounds;

        public byte drawShapes;

        public byte drawJoints;

        public byte drawJointExtras;

        public byte drawAABBs;

        public byte drawMass;

        public byte drawBodyNames;

        public byte drawContacts;

        public byte drawGraphColors;

        public byte drawContactNormals;

        public byte drawContactImpulses;

        public byte drawFrictionImpulses;

        public void* context;
    }

    public partial struct InlineArrays
    {
        [InlineArray(8)]
        public partial struct Vec2_8
        {
            public Vec2 Item0;
        }
    }

    public partial struct InlineArrays
    {
        [InlineArray(3)]
        public partial struct byte_3
        {
            public byte Item0;
        }
    }

    public partial struct InlineArrays
    {
        [InlineArray(2)]
        public partial struct ManifoldPoint_2
        {
            public ManifoldPoint Item0;
        }
    }

    public partial struct InlineArrays
    {
        [InlineArray(12)]
        public partial struct int_12
        {
            public int Item0;
        }
    }

    public const ulong B2_DEFAULT_CATEGORY_BITS = 1;

    public const ulong B2_DEFAULT_MASK_BITS = 18446744073709551615;

    public const int B2_HASH_INIT = 5381;

    public const int B2_MAX_POLYGON_VERTICES = 8;

    public const float B2_PI = 3.1415927f;

    public partial struct Version : IEquatable<Version>
    {
        public bool Equals(Version other)
        {
            fixed (Version* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Version)).SequenceEqual(new Span<byte>(&other, sizeof(Version)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Version other && Equals(other);
        }

        public static bool operator ==(Version left, Version right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Version left, Version right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Version* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Version)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Vec2 : IEquatable<Vec2>
    {
        public bool Equals(Vec2 other)
        {
            fixed (Vec2* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Vec2)).SequenceEqual(new Span<byte>(&other, sizeof(Vec2)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Vec2 other && Equals(other);
        }

        public static bool operator ==(Vec2 left, Vec2 right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Vec2 left, Vec2 right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Vec2* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Vec2)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct CosSin : IEquatable<CosSin>
    {
        public bool Equals(CosSin other)
        {
            fixed (CosSin* __self = &this)
            {
                return new Span<byte>(__self, sizeof(CosSin)).SequenceEqual(new Span<byte>(&other, sizeof(CosSin)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is CosSin other && Equals(other);
        }

        public static bool operator ==(CosSin left, CosSin right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(CosSin left, CosSin right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (CosSin* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(CosSin)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Rot : IEquatable<Rot>
    {
        public bool Equals(Rot other)
        {
            fixed (Rot* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Rot)).SequenceEqual(new Span<byte>(&other, sizeof(Rot)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Rot other && Equals(other);
        }

        public static bool operator ==(Rot left, Rot right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Rot left, Rot right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Rot* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Rot)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Transform : IEquatable<Transform>
    {
        public bool Equals(Transform other)
        {
            fixed (Transform* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Transform)).SequenceEqual(new Span<byte>(&other, sizeof(Transform)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Transform other && Equals(other);
        }

        public static bool operator ==(Transform left, Transform right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Transform left, Transform right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Transform* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Transform)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Mat22 : IEquatable<Mat22>
    {
        public bool Equals(Mat22 other)
        {
            fixed (Mat22* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Mat22)).SequenceEqual(new Span<byte>(&other, sizeof(Mat22)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Mat22 other && Equals(other);
        }

        public static bool operator ==(Mat22 left, Mat22 right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Mat22 left, Mat22 right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Mat22* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Mat22)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct AABB : IEquatable<AABB>
    {
        public bool Equals(AABB other)
        {
            fixed (AABB* __self = &this)
            {
                return new Span<byte>(__self, sizeof(AABB)).SequenceEqual(new Span<byte>(&other, sizeof(AABB)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is AABB other && Equals(other);
        }

        public static bool operator ==(AABB left, AABB right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(AABB left, AABB right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (AABB* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(AABB)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SimplexCache : IEquatable<SimplexCache>
    {
        public bool Equals(SimplexCache other)
        {
            fixed (SimplexCache* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SimplexCache)).SequenceEqual(new Span<byte>(&other, sizeof(SimplexCache)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SimplexCache other && Equals(other);
        }

        public static bool operator ==(SimplexCache left, SimplexCache right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SimplexCache left, SimplexCache right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SimplexCache* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SimplexCache)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Hull : IEquatable<Hull>
    {
        public bool Equals(Hull other)
        {
            fixed (Hull* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Hull)).SequenceEqual(new Span<byte>(&other, sizeof(Hull)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Hull other && Equals(other);
        }

        public static bool operator ==(Hull left, Hull right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Hull left, Hull right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Hull* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Hull)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct RayCastInput : IEquatable<RayCastInput>
    {
        public bool Equals(RayCastInput other)
        {
            fixed (RayCastInput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(RayCastInput)).SequenceEqual(new Span<byte>(&other, sizeof(RayCastInput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is RayCastInput other && Equals(other);
        }

        public static bool operator ==(RayCastInput left, RayCastInput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(RayCastInput left, RayCastInput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (RayCastInput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(RayCastInput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ShapeCastInput : IEquatable<ShapeCastInput>
    {
        public bool Equals(ShapeCastInput other)
        {
            fixed (ShapeCastInput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ShapeCastInput)).SequenceEqual(new Span<byte>(&other, sizeof(ShapeCastInput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ShapeCastInput other && Equals(other);
        }

        public static bool operator ==(ShapeCastInput left, ShapeCastInput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ShapeCastInput left, ShapeCastInput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ShapeCastInput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ShapeCastInput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct CastOutput : IEquatable<CastOutput>
    {
        public bool Equals(CastOutput other)
        {
            fixed (CastOutput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(CastOutput)).SequenceEqual(new Span<byte>(&other, sizeof(CastOutput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is CastOutput other && Equals(other);
        }

        public static bool operator ==(CastOutput left, CastOutput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(CastOutput left, CastOutput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (CastOutput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(CastOutput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct MassData : IEquatable<MassData>
    {
        public bool Equals(MassData other)
        {
            fixed (MassData* __self = &this)
            {
                return new Span<byte>(__self, sizeof(MassData)).SequenceEqual(new Span<byte>(&other, sizeof(MassData)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is MassData other && Equals(other);
        }

        public static bool operator ==(MassData left, MassData right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(MassData left, MassData right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (MassData* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(MassData)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Circle : IEquatable<Circle>
    {
        public bool Equals(Circle other)
        {
            fixed (Circle* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Circle)).SequenceEqual(new Span<byte>(&other, sizeof(Circle)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Circle other && Equals(other);
        }

        public static bool operator ==(Circle left, Circle right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Circle left, Circle right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Circle* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Circle)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Capsule : IEquatable<Capsule>
    {
        public bool Equals(Capsule other)
        {
            fixed (Capsule* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Capsule)).SequenceEqual(new Span<byte>(&other, sizeof(Capsule)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Capsule other && Equals(other);
        }

        public static bool operator ==(Capsule left, Capsule right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Capsule left, Capsule right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Capsule* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Capsule)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Polygon : IEquatable<Polygon>
    {
        public bool Equals(Polygon other)
        {
            fixed (Polygon* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Polygon)).SequenceEqual(new Span<byte>(&other, sizeof(Polygon)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Polygon other && Equals(other);
        }

        public static bool operator ==(Polygon left, Polygon right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Polygon left, Polygon right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Polygon* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Polygon)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Segment : IEquatable<Segment>
    {
        public bool Equals(Segment other)
        {
            fixed (Segment* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Segment)).SequenceEqual(new Span<byte>(&other, sizeof(Segment)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Segment other && Equals(other);
        }

        public static bool operator ==(Segment left, Segment right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Segment left, Segment right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Segment* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Segment)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ChainSegment : IEquatable<ChainSegment>
    {
        public bool Equals(ChainSegment other)
        {
            fixed (ChainSegment* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ChainSegment)).SequenceEqual(new Span<byte>(&other, sizeof(ChainSegment)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ChainSegment other && Equals(other);
        }

        public static bool operator ==(ChainSegment left, ChainSegment right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ChainSegment left, ChainSegment right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ChainSegment* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ChainSegment)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SegmentDistanceResult : IEquatable<SegmentDistanceResult>
    {
        public bool Equals(SegmentDistanceResult other)
        {
            fixed (SegmentDistanceResult* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SegmentDistanceResult)).SequenceEqual(new Span<byte>(&other, sizeof(SegmentDistanceResult)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SegmentDistanceResult other && Equals(other);
        }

        public static bool operator ==(SegmentDistanceResult left, SegmentDistanceResult right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SegmentDistanceResult left, SegmentDistanceResult right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SegmentDistanceResult* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SegmentDistanceResult)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ShapeProxy : IEquatable<ShapeProxy>
    {
        public bool Equals(ShapeProxy other)
        {
            fixed (ShapeProxy* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ShapeProxy)).SequenceEqual(new Span<byte>(&other, sizeof(ShapeProxy)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ShapeProxy other && Equals(other);
        }

        public static bool operator ==(ShapeProxy left, ShapeProxy right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ShapeProxy left, ShapeProxy right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ShapeProxy* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ShapeProxy)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct DistanceInput : IEquatable<DistanceInput>
    {
        public bool Equals(DistanceInput other)
        {
            fixed (DistanceInput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DistanceInput)).SequenceEqual(new Span<byte>(&other, sizeof(DistanceInput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DistanceInput other && Equals(other);
        }

        public static bool operator ==(DistanceInput left, DistanceInput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DistanceInput left, DistanceInput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DistanceInput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DistanceInput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct DistanceOutput : IEquatable<DistanceOutput>
    {
        public bool Equals(DistanceOutput other)
        {
            fixed (DistanceOutput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DistanceOutput)).SequenceEqual(new Span<byte>(&other, sizeof(DistanceOutput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DistanceOutput other && Equals(other);
        }

        public static bool operator ==(DistanceOutput left, DistanceOutput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DistanceOutput left, DistanceOutput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DistanceOutput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DistanceOutput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SimplexVertex : IEquatable<SimplexVertex>
    {
        public bool Equals(SimplexVertex other)
        {
            fixed (SimplexVertex* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SimplexVertex)).SequenceEqual(new Span<byte>(&other, sizeof(SimplexVertex)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SimplexVertex other && Equals(other);
        }

        public static bool operator ==(SimplexVertex left, SimplexVertex right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SimplexVertex left, SimplexVertex right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SimplexVertex* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SimplexVertex)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Simplex : IEquatable<Simplex>
    {
        public bool Equals(Simplex other)
        {
            fixed (Simplex* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Simplex)).SequenceEqual(new Span<byte>(&other, sizeof(Simplex)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Simplex other && Equals(other);
        }

        public static bool operator ==(Simplex left, Simplex right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Simplex left, Simplex right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Simplex* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Simplex)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ShapeCastPairInput : IEquatable<ShapeCastPairInput>
    {
        public bool Equals(ShapeCastPairInput other)
        {
            fixed (ShapeCastPairInput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ShapeCastPairInput)).SequenceEqual(new Span<byte>(&other, sizeof(ShapeCastPairInput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ShapeCastPairInput other && Equals(other);
        }

        public static bool operator ==(ShapeCastPairInput left, ShapeCastPairInput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ShapeCastPairInput left, ShapeCastPairInput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ShapeCastPairInput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ShapeCastPairInput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Sweep : IEquatable<Sweep>
    {
        public bool Equals(Sweep other)
        {
            fixed (Sweep* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Sweep)).SequenceEqual(new Span<byte>(&other, sizeof(Sweep)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Sweep other && Equals(other);
        }

        public static bool operator ==(Sweep left, Sweep right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Sweep left, Sweep right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Sweep* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Sweep)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct TOIInput : IEquatable<TOIInput>
    {
        public bool Equals(TOIInput other)
        {
            fixed (TOIInput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(TOIInput)).SequenceEqual(new Span<byte>(&other, sizeof(TOIInput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is TOIInput other && Equals(other);
        }

        public static bool operator ==(TOIInput left, TOIInput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(TOIInput left, TOIInput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (TOIInput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(TOIInput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct TOIOutput : IEquatable<TOIOutput>
    {
        public bool Equals(TOIOutput other)
        {
            fixed (TOIOutput* __self = &this)
            {
                return new Span<byte>(__self, sizeof(TOIOutput)).SequenceEqual(new Span<byte>(&other, sizeof(TOIOutput)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is TOIOutput other && Equals(other);
        }

        public static bool operator ==(TOIOutput left, TOIOutput right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(TOIOutput left, TOIOutput right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (TOIOutput* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(TOIOutput)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ManifoldPoint : IEquatable<ManifoldPoint>
    {
        public bool Equals(ManifoldPoint other)
        {
            fixed (ManifoldPoint* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ManifoldPoint)).SequenceEqual(new Span<byte>(&other, sizeof(ManifoldPoint)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ManifoldPoint other && Equals(other);
        }

        public static bool operator ==(ManifoldPoint left, ManifoldPoint right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ManifoldPoint left, ManifoldPoint right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ManifoldPoint* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ManifoldPoint)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Manifold : IEquatable<Manifold>
    {
        public bool Equals(Manifold other)
        {
            fixed (Manifold* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Manifold)).SequenceEqual(new Span<byte>(&other, sizeof(Manifold)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Manifold other && Equals(other);
        }

        public static bool operator ==(Manifold left, Manifold right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Manifold left, Manifold right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Manifold* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Manifold)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct DynamicTree : IEquatable<DynamicTree>
    {
        public bool Equals(DynamicTree other)
        {
            fixed (DynamicTree* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DynamicTree)).SequenceEqual(new Span<byte>(&other, sizeof(DynamicTree)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DynamicTree other && Equals(other);
        }

        public static bool operator ==(DynamicTree left, DynamicTree right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DynamicTree left, DynamicTree right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DynamicTree* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DynamicTree)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct TreeNode : IEquatable<TreeNode>
    {
        public bool Equals(TreeNode other)
        {
            fixed (TreeNode* __self = &this)
            {
                return new Span<byte>(__self, sizeof(TreeNode)).SequenceEqual(new Span<byte>(&other, sizeof(TreeNode)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is TreeNode other && Equals(other);
        }

        public static bool operator ==(TreeNode left, TreeNode right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(TreeNode left, TreeNode right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (TreeNode* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(TreeNode)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct TreeStats : IEquatable<TreeStats>
    {
        public bool Equals(TreeStats other)
        {
            fixed (TreeStats* __self = &this)
            {
                return new Span<byte>(__self, sizeof(TreeStats)).SequenceEqual(new Span<byte>(&other, sizeof(TreeStats)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is TreeStats other && Equals(other);
        }

        public static bool operator ==(TreeStats left, TreeStats right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(TreeStats left, TreeStats right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (TreeStats* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(TreeStats)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct WorldId : IEquatable<WorldId>
    {
        public bool Equals(WorldId other)
        {
            fixed (WorldId* __self = &this)
            {
                return new Span<byte>(__self, sizeof(WorldId)).SequenceEqual(new Span<byte>(&other, sizeof(WorldId)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is WorldId other && Equals(other);
        }

        public static bool operator ==(WorldId left, WorldId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(WorldId left, WorldId right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (WorldId* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(WorldId)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct BodyId : IEquatable<BodyId>
    {
        public bool Equals(BodyId other)
        {
            fixed (BodyId* __self = &this)
            {
                return new Span<byte>(__self, sizeof(BodyId)).SequenceEqual(new Span<byte>(&other, sizeof(BodyId)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is BodyId other && Equals(other);
        }

        public static bool operator ==(BodyId left, BodyId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BodyId left, BodyId right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (BodyId* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(BodyId)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ShapeId : IEquatable<ShapeId>
    {
        public bool Equals(ShapeId other)
        {
            fixed (ShapeId* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ShapeId)).SequenceEqual(new Span<byte>(&other, sizeof(ShapeId)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ShapeId other && Equals(other);
        }

        public static bool operator ==(ShapeId left, ShapeId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ShapeId left, ShapeId right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ShapeId* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ShapeId)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ChainId : IEquatable<ChainId>
    {
        public bool Equals(ChainId other)
        {
            fixed (ChainId* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ChainId)).SequenceEqual(new Span<byte>(&other, sizeof(ChainId)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ChainId other && Equals(other);
        }

        public static bool operator ==(ChainId left, ChainId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ChainId left, ChainId right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ChainId* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ChainId)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct JointId : IEquatable<JointId>
    {
        public bool Equals(JointId other)
        {
            fixed (JointId* __self = &this)
            {
                return new Span<byte>(__self, sizeof(JointId)).SequenceEqual(new Span<byte>(&other, sizeof(JointId)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is JointId other && Equals(other);
        }

        public static bool operator ==(JointId left, JointId right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(JointId left, JointId right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (JointId* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(JointId)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct RayResult : IEquatable<RayResult>
    {
        public bool Equals(RayResult other)
        {
            fixed (RayResult* __self = &this)
            {
                return new Span<byte>(__self, sizeof(RayResult)).SequenceEqual(new Span<byte>(&other, sizeof(RayResult)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is RayResult other && Equals(other);
        }

        public static bool operator ==(RayResult left, RayResult right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(RayResult left, RayResult right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (RayResult* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(RayResult)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct WorldDef : IEquatable<WorldDef>
    {
        public bool Equals(WorldDef other)
        {
            fixed (WorldDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(WorldDef)).SequenceEqual(new Span<byte>(&other, sizeof(WorldDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is WorldDef other && Equals(other);
        }

        public static bool operator ==(WorldDef left, WorldDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(WorldDef left, WorldDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (WorldDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(WorldDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct BodyDef : IEquatable<BodyDef>
    {
        public bool Equals(BodyDef other)
        {
            fixed (BodyDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(BodyDef)).SequenceEqual(new Span<byte>(&other, sizeof(BodyDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is BodyDef other && Equals(other);
        }

        public static bool operator ==(BodyDef left, BodyDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BodyDef left, BodyDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (BodyDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(BodyDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Filter : IEquatable<Filter>
    {
        public bool Equals(Filter other)
        {
            fixed (Filter* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Filter)).SequenceEqual(new Span<byte>(&other, sizeof(Filter)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Filter other && Equals(other);
        }

        public static bool operator ==(Filter left, Filter right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Filter left, Filter right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Filter* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Filter)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct QueryFilter : IEquatable<QueryFilter>
    {
        public bool Equals(QueryFilter other)
        {
            fixed (QueryFilter* __self = &this)
            {
                return new Span<byte>(__self, sizeof(QueryFilter)).SequenceEqual(new Span<byte>(&other, sizeof(QueryFilter)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is QueryFilter other && Equals(other);
        }

        public static bool operator ==(QueryFilter left, QueryFilter right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(QueryFilter left, QueryFilter right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (QueryFilter* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(QueryFilter)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ShapeDef : IEquatable<ShapeDef>
    {
        public bool Equals(ShapeDef other)
        {
            fixed (ShapeDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ShapeDef)).SequenceEqual(new Span<byte>(&other, sizeof(ShapeDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ShapeDef other && Equals(other);
        }

        public static bool operator ==(ShapeDef left, ShapeDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ShapeDef left, ShapeDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ShapeDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ShapeDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SurfaceMaterial : IEquatable<SurfaceMaterial>
    {
        public bool Equals(SurfaceMaterial other)
        {
            fixed (SurfaceMaterial* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SurfaceMaterial)).SequenceEqual(new Span<byte>(&other, sizeof(SurfaceMaterial)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SurfaceMaterial other && Equals(other);
        }

        public static bool operator ==(SurfaceMaterial left, SurfaceMaterial right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SurfaceMaterial left, SurfaceMaterial right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SurfaceMaterial* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SurfaceMaterial)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ChainDef : IEquatable<ChainDef>
    {
        public bool Equals(ChainDef other)
        {
            fixed (ChainDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ChainDef)).SequenceEqual(new Span<byte>(&other, sizeof(ChainDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ChainDef other && Equals(other);
        }

        public static bool operator ==(ChainDef left, ChainDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ChainDef left, ChainDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ChainDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ChainDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Profile : IEquatable<Profile>
    {
        public bool Equals(Profile other)
        {
            fixed (Profile* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Profile)).SequenceEqual(new Span<byte>(&other, sizeof(Profile)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Profile other && Equals(other);
        }

        public static bool operator ==(Profile left, Profile right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Profile left, Profile right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Profile* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Profile)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct Counters : IEquatable<Counters>
    {
        public bool Equals(Counters other)
        {
            fixed (Counters* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Counters)).SequenceEqual(new Span<byte>(&other, sizeof(Counters)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Counters other && Equals(other);
        }

        public static bool operator ==(Counters left, Counters right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Counters left, Counters right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Counters* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Counters)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct DistanceJointDef : IEquatable<DistanceJointDef>
    {
        public bool Equals(DistanceJointDef other)
        {
            fixed (DistanceJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DistanceJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(DistanceJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DistanceJointDef other && Equals(other);
        }

        public static bool operator ==(DistanceJointDef left, DistanceJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DistanceJointDef left, DistanceJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DistanceJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DistanceJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct MotorJointDef : IEquatable<MotorJointDef>
    {
        public bool Equals(MotorJointDef other)
        {
            fixed (MotorJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(MotorJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(MotorJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is MotorJointDef other && Equals(other);
        }

        public static bool operator ==(MotorJointDef left, MotorJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(MotorJointDef left, MotorJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (MotorJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(MotorJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct MouseJointDef : IEquatable<MouseJointDef>
    {
        public bool Equals(MouseJointDef other)
        {
            fixed (MouseJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(MouseJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(MouseJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is MouseJointDef other && Equals(other);
        }

        public static bool operator ==(MouseJointDef left, MouseJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(MouseJointDef left, MouseJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (MouseJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(MouseJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct NullJointDef : IEquatable<NullJointDef>
    {
        public bool Equals(NullJointDef other)
        {
            fixed (NullJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(NullJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(NullJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is NullJointDef other && Equals(other);
        }

        public static bool operator ==(NullJointDef left, NullJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(NullJointDef left, NullJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (NullJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(NullJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct PrismaticJointDef : IEquatable<PrismaticJointDef>
    {
        public bool Equals(PrismaticJointDef other)
        {
            fixed (PrismaticJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(PrismaticJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(PrismaticJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is PrismaticJointDef other && Equals(other);
        }

        public static bool operator ==(PrismaticJointDef left, PrismaticJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(PrismaticJointDef left, PrismaticJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (PrismaticJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(PrismaticJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct RevoluteJointDef : IEquatable<RevoluteJointDef>
    {
        public bool Equals(RevoluteJointDef other)
        {
            fixed (RevoluteJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(RevoluteJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(RevoluteJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is RevoluteJointDef other && Equals(other);
        }

        public static bool operator ==(RevoluteJointDef left, RevoluteJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(RevoluteJointDef left, RevoluteJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (RevoluteJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(RevoluteJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct WeldJointDef : IEquatable<WeldJointDef>
    {
        public bool Equals(WeldJointDef other)
        {
            fixed (WeldJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(WeldJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(WeldJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is WeldJointDef other && Equals(other);
        }

        public static bool operator ==(WeldJointDef left, WeldJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(WeldJointDef left, WeldJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (WeldJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(WeldJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct WheelJointDef : IEquatable<WheelJointDef>
    {
        public bool Equals(WheelJointDef other)
        {
            fixed (WheelJointDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(WheelJointDef)).SequenceEqual(new Span<byte>(&other, sizeof(WheelJointDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is WheelJointDef other && Equals(other);
        }

        public static bool operator ==(WheelJointDef left, WheelJointDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(WheelJointDef left, WheelJointDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (WheelJointDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(WheelJointDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ExplosionDef : IEquatable<ExplosionDef>
    {
        public bool Equals(ExplosionDef other)
        {
            fixed (ExplosionDef* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ExplosionDef)).SequenceEqual(new Span<byte>(&other, sizeof(ExplosionDef)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ExplosionDef other && Equals(other);
        }

        public static bool operator ==(ExplosionDef left, ExplosionDef right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ExplosionDef left, ExplosionDef right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ExplosionDef* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ExplosionDef)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SensorBeginTouchEvent : IEquatable<SensorBeginTouchEvent>
    {
        public bool Equals(SensorBeginTouchEvent other)
        {
            fixed (SensorBeginTouchEvent* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SensorBeginTouchEvent)).SequenceEqual(new Span<byte>(&other, sizeof(SensorBeginTouchEvent)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SensorBeginTouchEvent other && Equals(other);
        }

        public static bool operator ==(SensorBeginTouchEvent left, SensorBeginTouchEvent right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SensorBeginTouchEvent left, SensorBeginTouchEvent right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SensorBeginTouchEvent* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SensorBeginTouchEvent)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SensorEndTouchEvent : IEquatable<SensorEndTouchEvent>
    {
        public bool Equals(SensorEndTouchEvent other)
        {
            fixed (SensorEndTouchEvent* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SensorEndTouchEvent)).SequenceEqual(new Span<byte>(&other, sizeof(SensorEndTouchEvent)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SensorEndTouchEvent other && Equals(other);
        }

        public static bool operator ==(SensorEndTouchEvent left, SensorEndTouchEvent right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SensorEndTouchEvent left, SensorEndTouchEvent right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SensorEndTouchEvent* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SensorEndTouchEvent)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct SensorEvents : IEquatable<SensorEvents>
    {
        public bool Equals(SensorEvents other)
        {
            fixed (SensorEvents* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SensorEvents)).SequenceEqual(new Span<byte>(&other, sizeof(SensorEvents)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SensorEvents other && Equals(other);
        }

        public static bool operator ==(SensorEvents left, SensorEvents right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SensorEvents left, SensorEvents right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SensorEvents* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SensorEvents)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ContactBeginTouchEvent : IEquatable<ContactBeginTouchEvent>
    {
        public bool Equals(ContactBeginTouchEvent other)
        {
            fixed (ContactBeginTouchEvent* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ContactBeginTouchEvent)).SequenceEqual(new Span<byte>(&other, sizeof(ContactBeginTouchEvent)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactBeginTouchEvent other && Equals(other);
        }

        public static bool operator ==(ContactBeginTouchEvent left, ContactBeginTouchEvent right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ContactBeginTouchEvent left, ContactBeginTouchEvent right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ContactBeginTouchEvent* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ContactBeginTouchEvent)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ContactEndTouchEvent : IEquatable<ContactEndTouchEvent>
    {
        public bool Equals(ContactEndTouchEvent other)
        {
            fixed (ContactEndTouchEvent* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ContactEndTouchEvent)).SequenceEqual(new Span<byte>(&other, sizeof(ContactEndTouchEvent)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactEndTouchEvent other && Equals(other);
        }

        public static bool operator ==(ContactEndTouchEvent left, ContactEndTouchEvent right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ContactEndTouchEvent left, ContactEndTouchEvent right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ContactEndTouchEvent* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ContactEndTouchEvent)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ContactHitEvent : IEquatable<ContactHitEvent>
    {
        public bool Equals(ContactHitEvent other)
        {
            fixed (ContactHitEvent* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ContactHitEvent)).SequenceEqual(new Span<byte>(&other, sizeof(ContactHitEvent)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactHitEvent other && Equals(other);
        }

        public static bool operator ==(ContactHitEvent left, ContactHitEvent right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ContactHitEvent left, ContactHitEvent right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ContactHitEvent* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ContactHitEvent)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ContactEvents : IEquatable<ContactEvents>
    {
        public bool Equals(ContactEvents other)
        {
            fixed (ContactEvents* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ContactEvents)).SequenceEqual(new Span<byte>(&other, sizeof(ContactEvents)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactEvents other && Equals(other);
        }

        public static bool operator ==(ContactEvents left, ContactEvents right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ContactEvents left, ContactEvents right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ContactEvents* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ContactEvents)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct BodyMoveEvent : IEquatable<BodyMoveEvent>
    {
        public bool Equals(BodyMoveEvent other)
        {
            fixed (BodyMoveEvent* __self = &this)
            {
                return new Span<byte>(__self, sizeof(BodyMoveEvent)).SequenceEqual(new Span<byte>(&other, sizeof(BodyMoveEvent)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is BodyMoveEvent other && Equals(other);
        }

        public static bool operator ==(BodyMoveEvent left, BodyMoveEvent right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BodyMoveEvent left, BodyMoveEvent right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (BodyMoveEvent* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(BodyMoveEvent)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct BodyEvents : IEquatable<BodyEvents>
    {
        public bool Equals(BodyEvents other)
        {
            fixed (BodyEvents* __self = &this)
            {
                return new Span<byte>(__self, sizeof(BodyEvents)).SequenceEqual(new Span<byte>(&other, sizeof(BodyEvents)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is BodyEvents other && Equals(other);
        }

        public static bool operator ==(BodyEvents left, BodyEvents right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BodyEvents left, BodyEvents right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (BodyEvents* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(BodyEvents)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct ContactData : IEquatable<ContactData>
    {
        public bool Equals(ContactData other)
        {
            fixed (ContactData* __self = &this)
            {
                return new Span<byte>(__self, sizeof(ContactData)).SequenceEqual(new Span<byte>(&other, sizeof(ContactData)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactData other && Equals(other);
        }

        public static bool operator ==(ContactData left, ContactData right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ContactData left, ContactData right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (ContactData* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(ContactData)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct DebugDraw : IEquatable<DebugDraw>
    {
        public bool Equals(DebugDraw other)
        {
            fixed (DebugDraw* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DebugDraw)).SequenceEqual(new Span<byte>(&other, sizeof(DebugDraw)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DebugDraw other && Equals(other);
        }

        public static bool operator ==(DebugDraw left, DebugDraw right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DebugDraw left, DebugDraw right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DebugDraw* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DebugDraw)));
                return hash.ToHashCode();
            }
        }
    }

    public partial struct InlineArrays
    {
        public partial struct Vec2_8 : IEquatable<Vec2_8>
        {
            public bool Equals(Vec2_8 other)
            {
                fixed (Vec2_8* __self = &this)
                {
                    return new Span<byte>(__self, sizeof(Vec2_8)).SequenceEqual(new Span<byte>(&other, sizeof(Vec2_8)));
                }
            }

            public override bool Equals(object? obj)
            {
                return obj is Vec2_8 other && Equals(other);
            }

            public static bool operator ==(Vec2_8 left, Vec2_8 right)
            {
                return left.Equals(right);
            }

            public static bool operator !=(Vec2_8 left, Vec2_8 right)
            {
                return !(left == right);
            }

            public override int GetHashCode()
            {
                fixed (Vec2_8* __self = &this)
                {
                    HashCode hash = new();
                    hash.AddBytes(new Span<byte>(__self, sizeof(Vec2_8)));
                    return hash.ToHashCode();
                }
            }
        }
    }

    public partial struct InlineArrays
    {
        public partial struct byte_3 : IEquatable<byte_3>
        {
            public bool Equals(byte_3 other)
            {
                fixed (byte_3* __self = &this)
                {
                    return new Span<byte>(__self, sizeof(byte_3)).SequenceEqual(new Span<byte>(&other, sizeof(byte_3)));
                }
            }

            public override bool Equals(object? obj)
            {
                return obj is byte_3 other && Equals(other);
            }

            public static bool operator ==(byte_3 left, byte_3 right)
            {
                return left.Equals(right);
            }

            public static bool operator !=(byte_3 left, byte_3 right)
            {
                return !(left == right);
            }

            public override int GetHashCode()
            {
                fixed (byte_3* __self = &this)
                {
                    HashCode hash = new();
                    hash.AddBytes(new Span<byte>(__self, sizeof(byte_3)));
                    return hash.ToHashCode();
                }
            }
        }
    }

    public partial struct InlineArrays
    {
        public partial struct ManifoldPoint_2 : IEquatable<ManifoldPoint_2>
        {
            public bool Equals(ManifoldPoint_2 other)
            {
                fixed (ManifoldPoint_2* __self = &this)
                {
                    return new Span<byte>(__self, sizeof(ManifoldPoint_2)).SequenceEqual(new Span<byte>(&other, sizeof(ManifoldPoint_2)));
                }
            }

            public override bool Equals(object? obj)
            {
                return obj is ManifoldPoint_2 other && Equals(other);
            }

            public static bool operator ==(ManifoldPoint_2 left, ManifoldPoint_2 right)
            {
                return left.Equals(right);
            }

            public static bool operator !=(ManifoldPoint_2 left, ManifoldPoint_2 right)
            {
                return !(left == right);
            }

            public override int GetHashCode()
            {
                fixed (ManifoldPoint_2* __self = &this)
                {
                    HashCode hash = new();
                    hash.AddBytes(new Span<byte>(__self, sizeof(ManifoldPoint_2)));
                    return hash.ToHashCode();
                }
            }
        }
    }

    public partial struct InlineArrays
    {
        public partial struct int_12 : IEquatable<int_12>
        {
            public bool Equals(int_12 other)
            {
                fixed (int_12* __self = &this)
                {
                    return new Span<byte>(__self, sizeof(int_12)).SequenceEqual(new Span<byte>(&other, sizeof(int_12)));
                }
            }

            public override bool Equals(object? obj)
            {
                return obj is int_12 other && Equals(other);
            }

            public static bool operator ==(int_12 left, int_12 right)
            {
                return left.Equals(right);
            }

            public static bool operator !=(int_12 left, int_12 right)
            {
                return !(left == right);
            }

            public override int GetHashCode()
            {
                fixed (int_12* __self = &this)
                {
                    HashCode hash = new();
                    hash.AddBytes(new Span<byte>(__self, sizeof(int_12)));
                    return hash.ToHashCode();
                }
            }
        }
    }
}
#pragma warning restore CS9084
#nullable disable
