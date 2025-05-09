#nullable enable
using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

[assembly: DisableRuntimeMarshalling]
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
    public static extern bool AABBContains(AABB a, AABB b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Extents")]
    public static extern Vec2 AABBExtents(AABB a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_IsValid")]
    public static extern bool AABBIsValid(AABB aabb);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyAngularImpulse")]
    public static extern void BodyApplyAngularImpulse(BodyId bodyId, float impulse, bool wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyForce")]
    public static extern void BodyApplyForce(BodyId bodyId, Vec2 force, Vec2 point, bool wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyForceToCenter")]
    public static extern void BodyApplyForceToCenter(BodyId bodyId, Vec2 force, bool wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyLinearImpulse")]
    public static extern void BodyApplyLinearImpulse(BodyId bodyId, Vec2 impulse, Vec2 point, bool wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyLinearImpulseToCenter")]
    public static extern void BodyApplyLinearImpulseToCenter(BodyId bodyId, Vec2 impulse, bool wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyMassFromShapes")]
    public static extern void BodyApplyMassFromShapes(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyTorque")]
    public static extern void BodyApplyTorque(BodyId bodyId, float torque, bool wake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ComputeAABB")]
    public static extern AABB BodyComputeAABB(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Disable")]
    public static extern void BodyDisable(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Enable")]
    public static extern void BodyEnable(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableHitEvents")]
    public static extern void BodyEnableHitEvents(BodyId bodyId, bool enableHitEvents);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableSleep")]
    public static extern void BodyEnableSleep(BodyId bodyId, bool enableSleep);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularDamping")]
    public static extern float BodyGetAngularDamping(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularVelocity")]
    public static extern float BodyGetAngularVelocity(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAutomaticMass")]
    public static extern bool BodyGetAutomaticMass(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactCapacity")]
    public static extern int BodyGetContactCapacity(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactData")]
    public static extern int BodyGetContactData(BodyId bodyId, ContactData* contactData, int capacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetGravityScale")]
    public static extern float BodyGetGravityScale(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetInertiaTensor")]
    public static extern float BodyGetInertiaTensor(BodyId bodyId);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalVector")]
    public static extern Vec2 BodyGetLocalVector(BodyId bodyId, Vec2 worldVector);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetMass")]
    public static extern float BodyGetMass(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetMassData")]
    public static extern MassData BodyGetMassData(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetPosition")]
    public static extern Vec2 BodyGetPosition(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetRotation")]
    public static extern Rot BodyGetRotation(BodyId bodyId);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldCenterOfMass")]
    public static extern Vec2 BodyGetWorldCenterOfMass(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldPoint")]
    public static extern Vec2 BodyGetWorldPoint(BodyId bodyId, Vec2 localPoint);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldVector")]
    public static extern Vec2 BodyGetWorldVector(BodyId bodyId, Vec2 localVector);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsAwake")]
    public static extern bool BodyIsAwake(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsBullet")]
    public static extern bool BodyIsBullet(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsEnabled")]
    public static extern bool BodyIsEnabled(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsFixedRotation")]
    public static extern bool BodyIsFixedRotation(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsSleepEnabled")]
    public static extern bool BodyIsSleepEnabled(BodyId bodyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsValid")]
    public static extern bool BodyIsValid(BodyId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularDamping")]
    public static extern void BodySetAngularDamping(BodyId bodyId, float angularDamping);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularVelocity")]
    public static extern void BodySetAngularVelocity(BodyId bodyId, float angularVelocity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAutomaticMass")]
    public static extern void BodySetAutomaticMass(BodyId bodyId, bool automaticMass);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAwake")]
    public static extern void BodySetAwake(BodyId bodyId, bool awake);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetBullet")]
    public static extern void BodySetBullet(BodyId bodyId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetFixedRotation")]
    public static extern void BodySetFixedRotation(BodyId bodyId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetGravityScale")]
    public static extern void BodySetGravityScale(BodyId bodyId, float gravityScale);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetLinearDamping")]
    public static extern void BodySetLinearDamping(BodyId bodyId, float linearDamping);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetLinearVelocity")]
    public static extern void BodySetLinearVelocity(BodyId bodyId, Vec2 linearVelocity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetMassData")]
    public static extern void BodySetMassData(BodyId bodyId, MassData massData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetSleepThreshold")]
    public static extern void BodySetSleepThreshold(BodyId bodyId, float sleepVelocity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetTransform")]
    public static extern void BodySetTransform(BodyId bodyId, Vec2 position, Rot rotation);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetType")]
    public static extern void BodySetType(BodyId bodyId, BodyType type);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetUserData")]
    public static extern void BodySetUserData(BodyId bodyId, void* userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_IsValid")]
    public static extern bool ChainIsValid(ChainId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetFriction")]
    public static extern void ChainSetFriction(ChainId chainId, float friction);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSmoothSegmentAndCapsule")]
    public static extern Manifold CollideSmoothSegmentAndCapsule(SmoothSegment* smoothSegmentA, Transform xfA, Capsule* capsuleB, Transform xfB, DistanceCache* cache);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSmoothSegmentAndCircle")]
    public static extern Manifold CollideSmoothSegmentAndCircle(SmoothSegment* smoothSegmentA, Transform xfA, Circle* circleB, Transform xfB);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSmoothSegmentAndPolygon")]
    public static extern Manifold CollideSmoothSegmentAndPolygon(SmoothSegment* smoothSegmentA, Transform xfA, Polygon* polygonB, Transform xfB, DistanceCache* cache);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeHull")]
    public static extern Hull ComputeHull(Vec2* points, int count);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputePolygonAABB")]
    public static extern AABB ComputePolygonAABB(Polygon* shape, Transform transform);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputePolygonMass")]
    public static extern MassData ComputePolygonMass(Polygon* shape, float density);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreatePolygonShape")]
    public static extern ShapeId CreatePolygonShape(BodyId bodyId, ShapeDef* def, Polygon* polygon);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreatePrismaticJoint")]
    public static extern JointId CreatePrismaticJoint(WorldId worldId, PrismaticJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateRevoluteJoint")]
    public static extern JointId CreateRevoluteJoint(WorldId worldId, RevoluteJointDef* def);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateSegmentShape")]
    public static extern ShapeId CreateSegmentShape(BodyId bodyId, ShapeDef* def, Segment* segment);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateTimer")]
    public static extern Timer CreateTimer();

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultDistanceJointDef")]
    public static extern DistanceJointDef DefaultDistanceJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultFilter")]
    public static extern Filter DefaultFilter();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultMotorJointDef")]
    public static extern MotorJointDef DefaultMotorJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultMouseJointDef")]
    public static extern MouseJointDef DefaultMouseJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultPrismaticJointDef")]
    public static extern PrismaticJointDef DefaultPrismaticJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultQueryFilter")]
    public static extern QueryFilter DefaultQueryFilter();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultRevoluteJointDef")]
    public static extern RevoluteJointDef DefaultRevoluteJointDef();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultShapeDef")]
    public static extern ShapeDef DefaultShapeDef();

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
    public static extern void DestroyShape(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyWorld")]
    public static extern void DestroyWorld(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Distance")]
    public static extern float Distance(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableLimit")]
    public static extern void DistanceJointEnableLimit(JointId jointId, bool enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableMotor")]
    public static extern void DistanceJointEnableMotor(JointId jointId, bool enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableSpring")]
    public static extern void DistanceJointEnableSpring(JointId jointId, bool enableSpring);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetCurrentLength")]
    public static extern float DistanceJointGetCurrentLength(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetDampingRatio")]
    public static extern float DistanceJointGetDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetHertz")]
    public static extern float DistanceJointGetHertz(JointId jointId);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsLimitEnabled")]
    public static extern bool DistanceJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsMotorEnabled")]
    public static extern bool DistanceJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsSpringEnabled")]
    public static extern bool DistanceJointIsSpringEnabled(JointId jointId);

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
    public static extern int DynamicTreeCreateProxy(DynamicTree* tree, AABB aabb, uint categoryBits, int userData);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetMaxBalance")]
    public static extern int DynamicTreeGetMaxBalance(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetProxyCount")]
    public static extern int DynamicTreeGetProxyCount(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetUserData")]
    public static extern int DynamicTreeGetUserData(DynamicTree* tree, int proxyId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_MoveProxy")]
    public static extern void DynamicTreeMoveProxy(DynamicTree* tree, int proxyId, AABB aabb);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Query")]
    public static extern void DynamicTreeQuery(DynamicTree* tree, AABB aabb, uint maskBits, delegate* unmanaged<int, int, void*, bool> callback, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RayCast")]
    public static extern void DynamicTreeRayCast(DynamicTree* tree, RayCastInput* input, uint maskBits, delegate* unmanaged<RayCastInput*, int, int, void*, float> callback, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Rebuild")]
    public static extern int DynamicTreeRebuild(DynamicTree* tree, bool fullBuild);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RebuildBottomUp")]
    public static extern void DynamicTreeRebuildBottomUp(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShapeCast")]
    public static extern void DynamicTreeShapeCast(DynamicTree* tree, ShapeCastInput* input, uint maskBits, delegate* unmanaged<ShapeCastInput*, int, int, void*, float> callback, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShiftOrigin")]
    public static extern void DynamicTreeShiftOrigin(DynamicTree* tree, Vec2 newOrigin);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Validate")]
    public static extern void DynamicTreeValidate(DynamicTree* tree);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetByteCount")]
    public static extern int GetByteCount();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetInverse22")]
    public static extern Mat22 GetInverse22(Mat22 A);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthAndNormalize")]
    public static extern Vec2 GetLengthAndNormalize(float* length, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthUnitsPerMeter")]
    public static extern float GetLengthUnitsPerMeter();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMilliseconds")]
    public static extern float GetMilliseconds(Timer* timer);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMillisecondsAndReset")]
    public static extern float GetMillisecondsAndReset(Timer* timer);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetSweepTransform")]
    public static extern Transform GetSweepTransform(Sweep* sweep, float time);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetTicks")]
    public static extern long GetTicks(Timer* timer);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetVersion")]
    public static extern Version GetVersion();

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IntegrateRotation")]
    public static extern Rot IntegrateRotation(Rot q1, float deltaAngle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulRot")]
    public static extern Rot InvMulRot(Rot q, Rot r);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulTransforms")]
    public static extern Transform InvMulTransforms(Transform A, Transform B);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvRotateVector")]
    public static extern Vec2 InvRotateVector(Rot q, Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvTransformPoint")]
    public static extern Vec2 InvTransformPoint(Transform t, Vec2 p);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsNormalized")]
    public static extern bool IsNormalized(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValid")]
    public static extern bool IsValid(float a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidRay")]
    public static extern bool IsValidRay(RayCastInput* input);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetBodyA")]
    public static extern BodyId JointGetBodyA(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetBodyB")]
    public static extern BodyId JointGetBodyB(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetCollideConnected")]
    public static extern bool JointGetCollideConnected(JointId jointId);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_IsValid")]
    public static extern bool JointIsValid(JointId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_SetCollideConnected")]
    public static extern void JointSetCollideConnected(JointId jointId, bool shouldCollide);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeBox")]
    public static extern Polygon MakeBox(float hx, float hy);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetBox")]
    public static extern Polygon MakeOffsetBox(float hx, float hy, Vec2 center, float angle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetPolygon")]
    public static extern Polygon MakeOffsetPolygon(Hull* hull, float radius, Transform transform);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakePolygon")]
    public static extern Polygon MakePolygon(Hull* hull, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeProxy")]
    public static extern DistanceProxy MakeProxy(Vec2* vertices, int count, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeRot")]
    public static extern Rot MakeRot(float angle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeRoundedBox")]
    public static extern Polygon MakeRoundedBox(float hx, float hy, float radius);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeSquare")]
    public static extern Polygon MakeSquare(float h);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NormalizeChecked")]
    public static extern Vec2 NormalizeChecked(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NormalizeRot")]
    public static extern Rot NormalizeRot(Rot q);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInCapsule")]
    public static extern bool PointInCapsule(Vec2 point, Capsule* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInCircle")]
    public static extern bool PointInCircle(Vec2 point, Circle* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInPolygon")]
    public static extern bool PointInPolygon(Vec2 point, Polygon* shape);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableLimit")]
    public static extern void PrismaticJointEnableLimit(JointId jointId, bool enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableMotor")]
    public static extern void PrismaticJointEnableMotor(JointId jointId, bool enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableSpring")]
    public static extern void PrismaticJointEnableSpring(JointId jointId, bool enableSpring);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetLowerLimit")]
    public static extern float PrismaticJointGetLowerLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMaxMotorForce")]
    public static extern float PrismaticJointGetMaxMotorForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorForce")]
    public static extern float PrismaticJointGetMotorForce(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorSpeed")]
    public static extern float PrismaticJointGetMotorSpeed(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpringDampingRatio")]
    public static extern float PrismaticJointGetSpringDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpringHertz")]
    public static extern float PrismaticJointGetSpringHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetUpperLimit")]
    public static extern float PrismaticJointGetUpperLimit(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsLimitEnabled")]
    public static extern bool PrismaticJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsMotorEnabled")]
    public static extern bool PrismaticJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsSpringEnabled")]
    public static extern bool PrismaticJointIsSpringEnabled(JointId jointId);

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
    public static extern CastOutput RayCastSegment(RayCastInput* input, Segment* shape, bool oneSided);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RelativeAngle")]
    public static extern float RelativeAngle(Rot b, Rot a);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableLimit")]
    public static extern void RevoluteJointEnableLimit(JointId jointId, bool enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableMotor")]
    public static extern void RevoluteJointEnableMotor(JointId jointId, bool enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableSpring")]
    public static extern void RevoluteJointEnableSpring(JointId jointId, bool enableSpring);

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
    public static extern bool RevoluteJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_IsMotorEnabled")]
    public static extern bool RevoluteJointIsMotorEnabled(JointId jointId);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_IsValid")]
    public static extern bool RotIsValid(Rot q);

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
    public static extern bool ShapeAreContactEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreHitEventsEnabled")]
    public static extern bool ShapeAreHitEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_ArePreSolveEventsEnabled")]
    public static extern bool ShapeArePreSolveEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreSensorEventsEnabled")]
    public static extern bool ShapeAreSensorEventsEnabled(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableContactEvents")]
    public static extern void ShapeEnableContactEvents(ShapeId shapeId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableHitEvents")]
    public static extern void ShapeEnableHitEvents(ShapeId shapeId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnablePreSolveEvents")]
    public static extern void ShapeEnablePreSolveEvents(ShapeId shapeId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableSensorEvents")]
    public static extern void ShapeEnableSensorEvents(ShapeId shapeId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetAABB")]
    public static extern AABB ShapeGetAABB(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetBody")]
    public static extern BodyId ShapeGetBody(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetCapsule")]
    public static extern Capsule ShapeGetCapsule(ShapeId shapeId);

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

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetParentChain")]
    public static extern ChainId ShapeGetParentChain(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetPolygon")]
    public static extern Polygon ShapeGetPolygon(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetRestitution")]
    public static extern float ShapeGetRestitution(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSegment")]
    public static extern Segment ShapeGetSegment(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSmoothSegment")]
    public static extern SmoothSegment ShapeGetSmoothSegment(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetType")]
    public static extern ShapeType ShapeGetType(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetUserData")]
    public static extern void* ShapeGetUserData(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_IsSensor")]
    public static extern bool ShapeIsSensor(ShapeId shapeId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_IsValid")]
    public static extern bool ShapeIsValid(ShapeId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_RayCast")]
    public static extern CastOutput ShapeRayCast(ShapeId shapeId, Vec2 origin, Vec2 translation);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetCapsule")]
    public static extern void ShapeSetCapsule(ShapeId shapeId, Capsule* capsule);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetCircle")]
    public static extern void ShapeSetCircle(ShapeId shapeId, Circle* circle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetDensity")]
    public static extern void ShapeSetDensity(ShapeId shapeId, float density);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetFilter")]
    public static extern void ShapeSetFilter(ShapeId shapeId, Filter filter);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetFriction")]
    public static extern void ShapeSetFriction(ShapeId shapeId, float friction);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetPolygon")]
    public static extern void ShapeSetPolygon(ShapeId shapeId, Polygon* polygon);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetRestitution")]
    public static extern void ShapeSetRestitution(ShapeId shapeId, float restitution);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetSegment")]
    public static extern void ShapeSetSegment(ShapeId shapeId, Segment* segment);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetUserData")]
    public static extern void ShapeSetUserData(ShapeId shapeId, void* userData);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_TestPoint")]
    public static extern bool ShapeTestPoint(ShapeId shapeId, Vec2 point);

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
    public static extern DistanceOutput ShapeDistance(DistanceCache* cache, DistanceInput* input, Simplex* simplexes, int simplexCapacity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SleepMilliseconds")]
    public static extern void SleepMilliseconds(int milliseconds);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Solve22")]
    public static extern Vec2 Solve22(Mat22 A, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Sub")]
    public static extern Vec2 Sub(Vec2 a, Vec2 b);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TimeOfImpact")]
    public static extern TOIOutput TimeOfImpact(TOIInput* input);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPoint")]
    public static extern Vec2 TransformPoint(Transform t, Vec2 p);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPolygon")]
    public static extern Polygon TransformPolygon(Transform transform, Polygon* polygon);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2UnwindAngle")]
    public static extern float UnwindAngle(float angle);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ValidateHull")]
    public static extern bool ValidateHull(Hull* hull);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Vec2_IsValid")]
    public static extern bool Vec2IsValid(Vec2 v);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetAngularDampingRatio")]
    public static extern float WeldJointGetAngularDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetAngularHertz")]
    public static extern float WeldJointGetAngularHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetLinearDampingRatio")]
    public static extern float WeldJointGetLinearDampingRatio(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetLinearHertz")]
    public static extern float WeldJointGetLinearHertz(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetAngularDampingRatio")]
    public static extern void WeldJointSetAngularDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetAngularHertz")]
    public static extern void WeldJointSetAngularHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetLinearDampingRatio")]
    public static extern void WeldJointSetLinearDampingRatio(JointId jointId, float dampingRatio);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetLinearHertz")]
    public static extern void WeldJointSetLinearHertz(JointId jointId, float hertz);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableLimit")]
    public static extern void WheelJointEnableLimit(JointId jointId, bool enableLimit);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableMotor")]
    public static extern void WheelJointEnableMotor(JointId jointId, bool enableMotor);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableSpring")]
    public static extern void WheelJointEnableSpring(JointId jointId, bool enableSpring);

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
    public static extern bool WheelJointIsLimitEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsMotorEnabled")]
    public static extern bool WheelJointIsMotorEnabled(JointId jointId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsSpringEnabled")]
    public static extern bool WheelJointIsSpringEnabled(JointId jointId);

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
    public static extern void WorldCastCapsule(WorldId worldId, Capsule* capsule, Transform originTransform, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastCircle")]
    public static extern void WorldCastCircle(WorldId worldId, Circle* circle, Transform originTransform, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastPolygon")]
    public static extern void WorldCastPolygon(WorldId worldId, Polygon* polygon, Transform originTransform, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastRay")]
    public static extern void WorldCastRay(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter, delegate* unmanaged<ShapeId, Vec2, Vec2, float, void*, float> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastRayClosest")]
    public static extern RayResult WorldCastRayClosest(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Draw")]
    public static extern void WorldDraw(WorldId worldId, DebugDraw* draw);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_DumpMemoryStats")]
    public static extern void WorldDumpMemoryStats(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableContinuous")]
    public static extern void WorldEnableContinuous(WorldId worldId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableSleeping")]
    public static extern void WorldEnableSleeping(WorldId worldId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableWarmStarting")]
    public static extern void WorldEnableWarmStarting(WorldId worldId, bool flag);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Explode")]
    public static extern void WorldExplode(WorldId worldId, Vec2 position, float radius, float impulse);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetBodyEvents")]
    public static extern BodyEvents WorldGetBodyEvents(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetContactEvents")]
    public static extern ContactEvents WorldGetContactEvents(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetCounters")]
    public static extern Counters WorldGetCounters(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetGravity")]
    public static extern Vec2 WorldGetGravity(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetProfile")]
    public static extern Profile WorldGetProfile(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetSensorEvents")]
    public static extern SensorEvents WorldGetSensorEvents(WorldId worldId);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsValid")]
    public static extern bool WorldIsValid(WorldId id);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapAABB")]
    public static extern void WorldOverlapAABB(WorldId worldId, AABB aabb, QueryFilter filter, delegate* unmanaged<ShapeId, void*, bool> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCapsule")]
    public static extern void WorldOverlapCapsule(WorldId worldId, Capsule* capsule, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, bool> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCircle")]
    public static extern void WorldOverlapCircle(WorldId worldId, Circle* circle, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, bool> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapPolygon")]
    public static extern void WorldOverlapPolygon(WorldId worldId, Polygon* polygon, Transform transform, QueryFilter filter, delegate* unmanaged<ShapeId, void*, bool> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetContactTuning")]
    public static extern void WorldSetContactTuning(WorldId worldId, float hertz, float dampingRatio, float pushVelocity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetCustomFilterCallback")]
    public static extern void WorldSetCustomFilterCallback(WorldId worldId, delegate* unmanaged<ShapeId, ShapeId, void*, bool> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetGravity")]
    public static extern void WorldSetGravity(WorldId worldId, Vec2 gravity);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetHitEventThreshold")]
    public static extern void WorldSetHitEventThreshold(WorldId worldId, float value);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetPreSolveCallback")]
    public static extern void WorldSetPreSolveCallback(WorldId worldId, delegate* unmanaged<ShapeId, ShapeId, Manifold*, void*, bool> fcn, void* context);

    [DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetRestitutionThreshold")]
    public static extern void WorldSetRestitutionThreshold(WorldId worldId, float value);

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
        colorDarkGoldenrod = 12092939,
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
        colorFirebrick = 11674146,
        colorFloralWhite = 16775920,
        colorForestGreen = 2263842,
        colorFuchsia = 16711935,
        colorGainsboro = 14474460,
        colorGhostWhite = 16316671,
        colorGold = 16766720,
        colorGoldenrod = 14329120,
        colorGray = 12500670,
        colorGray1 = 1710618,
        colorGray2 = 3355443,
        colorGray3 = 5066061,
        colorGray4 = 6710886,
        colorGray5 = 8355711,
        colorGray6 = 10066329,
        colorGray7 = 11776947,
        colorGray8 = 13421772,
        colorGray9 = 15066597,
        colorGreen = 65280,
        colorGreenYellow = 11403055,
        colorHoneydew = 15794160,
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
        colorLightGoldenrod = 15654274,
        colorLightGoldenrodYellow = 16448210,
        colorLightGray = 13882323,
        colorLightGreen = 9498256,
        colorLightPink = 16758465,
        colorLightSalmon = 16752762,
        colorLightSeaGreen = 2142890,
        colorLightSkyBlue = 8900346,
        colorLightSlateBlue = 8679679,
        colorLightSlateGray = 7833753,
        colorLightSteelBlue = 11584734,
        colorLightYellow = 16777184,
        colorLime = 65280,
        colorLimeGreen = 3329330,
        colorLinen = 16445670,
        colorMagenta = 16711935,
        colorMaroon = 11546720,
        colorMediumAquamarine = 6737322,
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
        colorNavyBlue = 128,
        colorOldLace = 16643558,
        colorOlive = 8421376,
        colorOliveDrab = 7048739,
        colorOrange = 16753920,
        colorOrangeRed = 16729344,
        colorOrchid = 14315734,
        colorPaleGoldenrod = 15657130,
        colorPaleGreen = 10025880,
        colorPaleTurquoise = 11529966,
        colorPaleVioletRed = 14381203,
        colorPapayaWhip = 16773077,
        colorPeachPuff = 16767673,
        colorPeru = 13468991,
        colorPink = 16761035,
        colorPlum = 14524637,
        colorPowderBlue = 11591910,
        colorPurple = 10494192,
        colorRebeccaPurple = 6697881,
        colorRed = 16711680,
        colorRosyBrown = 12357519,
        colorRoyalBlue = 4286945,
        colorSaddleBrown = 9127187,
        colorSalmon = 16416882,
        colorSandyBrown = 16032864,
        colorSeaGreen = 3050327,
        colorSeashell = 16774638,
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
        colorVioletRed = 13639824,
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
        prismaticJoint = 3,
        revoluteJoint = 4,
        weldJoint = 5,
        wheelJoint = 6
    }

    public enum ShapeType : uint
    {
        circleShape = 0,
        capsuleShape = 1,
        segmentShape = 2,
        polygonShape = 3,
        smoothSegmentShape = 4,
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

    public const HexColor colorDarkGoldenrod = HexColor.colorDarkGoldenrod;

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

    public const HexColor colorFirebrick = HexColor.colorFirebrick;

    public const HexColor colorFloralWhite = HexColor.colorFloralWhite;

    public const HexColor colorForestGreen = HexColor.colorForestGreen;

    public const HexColor colorFuchsia = HexColor.colorFuchsia;

    public const HexColor colorGainsboro = HexColor.colorGainsboro;

    public const HexColor colorGhostWhite = HexColor.colorGhostWhite;

    public const HexColor colorGold = HexColor.colorGold;

    public const HexColor colorGoldenrod = HexColor.colorGoldenrod;

    public const HexColor colorGray = HexColor.colorGray;

    public const HexColor colorGray1 = HexColor.colorGray1;

    public const HexColor colorGray2 = HexColor.colorGray2;

    public const HexColor colorGray3 = HexColor.colorGray3;

    public const HexColor colorGray4 = HexColor.colorGray4;

    public const HexColor colorGray5 = HexColor.colorGray5;

    public const HexColor colorGray6 = HexColor.colorGray6;

    public const HexColor colorGray7 = HexColor.colorGray7;

    public const HexColor colorGray8 = HexColor.colorGray8;

    public const HexColor colorGray9 = HexColor.colorGray9;

    public const HexColor colorGreen = HexColor.colorGreen;

    public const HexColor colorGreenYellow = HexColor.colorGreenYellow;

    public const HexColor colorHoneydew = HexColor.colorHoneydew;

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

    public const HexColor colorLightGoldenrod = HexColor.colorLightGoldenrod;

    public const HexColor colorLightGoldenrodYellow = HexColor.colorLightGoldenrodYellow;

    public const HexColor colorLightGray = HexColor.colorLightGray;

    public const HexColor colorLightGreen = HexColor.colorLightGreen;

    public const HexColor colorLightPink = HexColor.colorLightPink;

    public const HexColor colorLightSalmon = HexColor.colorLightSalmon;

    public const HexColor colorLightSeaGreen = HexColor.colorLightSeaGreen;

    public const HexColor colorLightSkyBlue = HexColor.colorLightSkyBlue;

    public const HexColor colorLightSlateBlue = HexColor.colorLightSlateBlue;

    public const HexColor colorLightSlateGray = HexColor.colorLightSlateGray;

    public const HexColor colorLightSteelBlue = HexColor.colorLightSteelBlue;

    public const HexColor colorLightYellow = HexColor.colorLightYellow;

    public const HexColor colorLime = HexColor.colorLime;

    public const HexColor colorLimeGreen = HexColor.colorLimeGreen;

    public const HexColor colorLinen = HexColor.colorLinen;

    public const HexColor colorMagenta = HexColor.colorMagenta;

    public const HexColor colorMaroon = HexColor.colorMaroon;

    public const HexColor colorMediumAquamarine = HexColor.colorMediumAquamarine;

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

    public const HexColor colorNavyBlue = HexColor.colorNavyBlue;

    public const HexColor colorOldLace = HexColor.colorOldLace;

    public const HexColor colorOlive = HexColor.colorOlive;

    public const HexColor colorOliveDrab = HexColor.colorOliveDrab;

    public const HexColor colorOrange = HexColor.colorOrange;

    public const HexColor colorOrangeRed = HexColor.colorOrangeRed;

    public const HexColor colorOrchid = HexColor.colorOrchid;

    public const HexColor colorPaleGoldenrod = HexColor.colorPaleGoldenrod;

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

    public const HexColor colorSeashell = HexColor.colorSeashell;

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

    public const HexColor colorVioletRed = HexColor.colorVioletRed;

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

    public const JointType prismaticJoint = JointType.prismaticJoint;

    public const JointType revoluteJoint = JointType.revoluteJoint;

    public const JointType weldJoint = JointType.weldJoint;

    public const JointType wheelJoint = JointType.wheelJoint;

    public const ShapeType circleShape = ShapeType.circleShape;

    public const ShapeType capsuleShape = ShapeType.capsuleShape;

    public const ShapeType segmentShape = ShapeType.segmentShape;

    public const ShapeType polygonShape = ShapeType.polygonShape;

    public const ShapeType smoothSegmentShape = ShapeType.smoothSegmentShape;

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

    public partial struct Timer
    {
        public ulong start_sec;

        public ulong start_usec;
    }

    public partial struct Vec2
    {
        public float x;

        public float y;
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

    public partial struct DistanceCache
    {
        public ushort count;

        public InlineArrays.byte_3 indexA;

        public InlineArrays.byte_3 indexB;
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

    public partial struct SmoothSegment
    {
        public Vec2 ghost1;

        public Segment segment;

        public Vec2 ghost2;

        public int chainId;
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

        public bool hit;
    }

    public partial struct MassData
    {
        public float mass;

        public Vec2 center;

        public float rotationalInertia;
    }

    public partial struct SegmentDistanceResult
    {
        public Vec2 closest1;

        public Vec2 closest2;

        public float fraction1;

        public float fraction2;

        public float distanceSquared;
    }

    public partial struct DistanceProxy
    {
        public InlineArrays.Vec2_8 points;

        public int count;

        public float radius;
    }

    public partial struct DistanceInput
    {
        public DistanceProxy proxyA;

        public DistanceProxy proxyB;

        public Transform transformA;

        public Transform transformB;

        public bool useRadii;
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
        public DistanceProxy proxyA;

        public DistanceProxy proxyB;

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
        public DistanceProxy proxyA;

        public DistanceProxy proxyB;

        public Sweep sweepA;

        public Sweep sweepB;

        public float tMax;
    }

    public partial struct TOIOutput
    {
        public TOIState state;

        public float t;
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

        public bool persisted;
    }

    public partial struct Manifold
    {
        public InlineArrays.ManifoldPoint_2 points;

        public Vec2 normal;

        public int pointCount;
    }

    public partial struct TreeNode
    {
        public AABB aabb;

        public uint categoryBits;

        public TreeNode.AnonymousRecord_collision_L608_C2 TreeNode_AnonymousRecord_collision_L608_C2_Field;

        public int child1;

        public int child2;

        public int userData;

        public short height;

        public bool enlarged;

        public InlineArrays.byte_9 pad;

        public ref int parent => ref TreeNode_AnonymousRecord_collision_L608_C2_Field.parent;

        public ref int next => ref TreeNode_AnonymousRecord_collision_L608_C2_Field.next;
    }

    public partial struct TreeNode
    {
        [StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
        public partial struct AnonymousRecord_collision_L608_C2
        {
            [System.Runtime.InteropServices.FieldOffset(0)]
            public int parent;

            [System.Runtime.InteropServices.FieldOffset(0)]
            public int next;
        }
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

    public partial struct WorldId
    {
        public ushort index1;

        public ushort revision;
    }

    public partial struct BodyId
    {
        public int index1;

        public ushort world0;

        public ushort revision;
    }

    public partial struct ShapeId
    {
        public int index1;

        public ushort world0;

        public ushort revision;
    }

    public partial struct JointId
    {
        public int index1;

        public ushort world0;

        public ushort revision;
    }

    public partial struct ChainId
    {
        public int index1;

        public ushort world0;

        public ushort revision;
    }

    public partial struct RayResult
    {
        public ShapeId shapeId;

        public Vec2 point;

        public Vec2 normal;

        public float fraction;

        public bool hit;
    }

    public partial struct WorldDef
    {
        public Vec2 gravity;

        public float restitutionThreshold;

        public float contactPushoutVelocity;

        public float hitEventThreshold;

        public float contactHertz;

        public float contactDampingRatio;

        public float jointHertz;

        public float jointDampingRatio;

        public float maximumLinearVelocity;

        public bool enableSleep;

        public bool enableContinous;

        public int workerCount;

        public delegate* unmanaged<delegate* unmanaged<int, int, uint, void*, void> , int, int, void*, void*, void*> enqueueTask;

        public delegate* unmanaged<void*, void*, void> finishTask;

        public void* userTaskContext;

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

        public void* userData;

        public bool enableSleep;

        public bool isAwake;

        public bool fixedRotation;

        public bool isBullet;

        public bool isEnabled;

        public bool automaticMass;

        public bool allowFastRotation;

        public int internalValue;
    }

    public partial struct Filter
    {
        public uint categoryBits;

        public uint maskBits;

        public int groupIndex;
    }

    public partial struct QueryFilter
    {
        public uint categoryBits;

        public uint maskBits;
    }

    public partial struct ShapeDef
    {
        public void* userData;

        public float friction;

        public float restitution;

        public float density;

        public Filter filter;

        public uint customColor;

        public bool isSensor;

        public bool enableSensorEvents;

        public bool enableContactEvents;

        public bool enableHitEvents;

        public bool enablePreSolveEvents;

        public bool forceContactCreation;

        public int internalValue;
    }

    public partial struct ChainDef
    {
        public void* userData;

        public Vec2* points;

        public int count;

        public float friction;

        public float restitution;

        public Filter filter;

        public bool isLoop;

        public int internalValue;
    }

    public partial struct Profile
    {
        public float step;

        public float pairs;

        public float collide;

        public float solve;

        public float buildIslands;

        public float solveConstraints;

        public float prepareTasks;

        public float solverTasks;

        public float prepareConstraints;

        public float integrateVelocities;

        public float warmStart;

        public float solveVelocities;

        public float integratePositions;

        public float relaxVelocities;

        public float applyRestitution;

        public float storeImpulses;

        public float finalizeBodies;

        public float splitIslands;

        public float sleepIslands;

        public float hitEvents;

        public float broadphase;

        public float continuous;
    }

    public partial struct Counters
    {
        public int staticBodyCount;

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

        public bool enableSpring;

        public float hertz;

        public float dampingRatio;

        public bool enableLimit;

        public float minLength;

        public float maxLength;

        public bool enableMotor;

        public float maxMotorForce;

        public float motorSpeed;

        public bool collideConnected;

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

        public bool collideConnected;

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

        public bool collideConnected;

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

        public bool enableSpring;

        public float hertz;

        public float dampingRatio;

        public bool enableLimit;

        public float lowerTranslation;

        public float upperTranslation;

        public bool enableMotor;

        public float maxMotorForce;

        public float motorSpeed;

        public bool collideConnected;

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

        public bool enableSpring;

        public float hertz;

        public float dampingRatio;

        public bool enableLimit;

        public float lowerAngle;

        public float upperAngle;

        public bool enableMotor;

        public float maxMotorTorque;

        public float motorSpeed;

        public float drawSize;

        public bool collideConnected;

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

        public bool collideConnected;

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

        public bool enableSpring;

        public float hertz;

        public float dampingRatio;

        public bool enableLimit;

        public float lowerTranslation;

        public float upperTranslation;

        public bool enableMotor;

        public float maxMotorTorque;

        public float motorSpeed;

        public bool collideConnected;

        public void* userData;

        public int internalValue;
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

        public bool fellAsleep;
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

        public delegate* unmanaged<Vec2, Vec2, float, HexColor, void*, void> DrawCapsule;

        public delegate* unmanaged<Vec2, Vec2, float, HexColor, void*, void> DrawSolidCapsule;

        public delegate* unmanaged<Vec2, Vec2, HexColor, void*, void> DrawSegment;

        public delegate* unmanaged<Transform, void*, void> DrawTransform;

        public delegate* unmanaged<Vec2, float, HexColor, void*, void> DrawPoint;

        public delegate* unmanaged<Vec2, byte*, void*, void> DrawString;

        public AABB drawingBounds;

        public bool useDrawingBounds;

        public bool drawShapes;

        public bool drawJoints;

        public bool drawJointExtras;

        public bool drawAABBs;

        public bool drawMass;

        public bool drawContacts;

        public bool drawGraphColors;

        public bool drawContactNormals;

        public bool drawContactImpulses;

        public bool drawFrictionImpulses;

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
        [InlineArray(9)]
        public partial struct byte_9
        {
            public byte Item0;
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

    public const int defaultCategoryBits = 1;

    public const uint defaultMaskBits = 4294967295;

    public const int maxPolygonVertices = 8;

    public const float pi = 3.1415927f;

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

    public partial struct Timer : IEquatable<Timer>
    {
        public bool Equals(Timer other)
        {
            fixed (Timer* __self = &this)
            {
                return new Span<byte>(__self, sizeof(Timer)).SequenceEqual(new Span<byte>(&other, sizeof(Timer)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is Timer other && Equals(other);
        }

        public static bool operator ==(Timer left, Timer right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Timer left, Timer right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (Timer* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(Timer)));
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

    public partial struct DistanceCache : IEquatable<DistanceCache>
    {
        public bool Equals(DistanceCache other)
        {
            fixed (DistanceCache* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DistanceCache)).SequenceEqual(new Span<byte>(&other, sizeof(DistanceCache)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DistanceCache other && Equals(other);
        }

        public static bool operator ==(DistanceCache left, DistanceCache right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DistanceCache left, DistanceCache right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DistanceCache* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DistanceCache)));
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

    public partial struct SmoothSegment : IEquatable<SmoothSegment>
    {
        public bool Equals(SmoothSegment other)
        {
            fixed (SmoothSegment* __self = &this)
            {
                return new Span<byte>(__self, sizeof(SmoothSegment)).SequenceEqual(new Span<byte>(&other, sizeof(SmoothSegment)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is SmoothSegment other && Equals(other);
        }

        public static bool operator ==(SmoothSegment left, SmoothSegment right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(SmoothSegment left, SmoothSegment right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (SmoothSegment* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(SmoothSegment)));
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

    public partial struct DistanceProxy : IEquatable<DistanceProxy>
    {
        public bool Equals(DistanceProxy other)
        {
            fixed (DistanceProxy* __self = &this)
            {
                return new Span<byte>(__self, sizeof(DistanceProxy)).SequenceEqual(new Span<byte>(&other, sizeof(DistanceProxy)));
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is DistanceProxy other && Equals(other);
        }

        public static bool operator ==(DistanceProxy left, DistanceProxy right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(DistanceProxy left, DistanceProxy right)
        {
            return !(left == right);
        }

        public override int GetHashCode()
        {
            fixed (DistanceProxy* __self = &this)
            {
                HashCode hash = new();
                hash.AddBytes(new Span<byte>(__self, sizeof(DistanceProxy)));
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

    public partial struct TreeNode
    {
        public partial struct AnonymousRecord_collision_L608_C2 : IEquatable<AnonymousRecord_collision_L608_C2>
        {
            public bool Equals(AnonymousRecord_collision_L608_C2 other)
            {
                fixed (AnonymousRecord_collision_L608_C2* __self = &this)
                {
                    return new Span<byte>(__self, sizeof(AnonymousRecord_collision_L608_C2)).SequenceEqual(new Span<byte>(&other, sizeof(AnonymousRecord_collision_L608_C2)));
                }
            }

            public override bool Equals(object? obj)
            {
                return obj is AnonymousRecord_collision_L608_C2 other && Equals(other);
            }

            public static bool operator ==(AnonymousRecord_collision_L608_C2 left, AnonymousRecord_collision_L608_C2 right)
            {
                return left.Equals(right);
            }

            public static bool operator !=(AnonymousRecord_collision_L608_C2 left, AnonymousRecord_collision_L608_C2 right)
            {
                return !(left == right);
            }

            public override int GetHashCode()
            {
                fixed (AnonymousRecord_collision_L608_C2* __self = &this)
                {
                    HashCode hash = new();
                    hash.AddBytes(new Span<byte>(__self, sizeof(AnonymousRecord_collision_L608_C2)));
                    return hash.ToHashCode();
                }
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
        public partial struct byte_9 : IEquatable<byte_9>
        {
            public bool Equals(byte_9 other)
            {
                fixed (byte_9* __self = &this)
                {
                    return new Span<byte>(__self, sizeof(byte_9)).SequenceEqual(new Span<byte>(&other, sizeof(byte_9)));
                }
            }

            public override bool Equals(object? obj)
            {
                return obj is byte_9 other && Equals(other);
            }

            public static bool operator ==(byte_9 left, byte_9 right)
            {
                return left.Equals(right);
            }

            public static bool operator !=(byte_9 left, byte_9 right)
            {
                return !(left == right);
            }

            public override int GetHashCode()
            {
                fixed (byte_9* __self = &this)
                {
                    HashCode hash = new();
                    hash.AddBytes(new Span<byte>(__self, sizeof(byte_9)));
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
#nullable disable
