#nullable enable
#pragma warning disable CS9084
namespace Box2D.NET.Bindings
{
    public static unsafe partial class B2
    {
        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Center", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 AABBCenter(AABB a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Contains", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte AABBContains(AABB a, AABB b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Extents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 AABBExtents(AABB a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte AABBIsValid(AABB aabb);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AABB_Union", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB AABBUnion(AABB a, AABB b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Abs", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Abs(Vec2 a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AbsFloat", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float AbsFloat(float a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2AbsInt", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int AbsInt(int a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Add", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Add(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyAngularImpulse", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyAngularImpulse(BodyId bodyId, float impulse, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyForce(BodyId bodyId, Vec2 force, Vec2 point, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyForceToCenter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyForceToCenter(BodyId bodyId, Vec2 force, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyLinearImpulse", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyLinearImpulse(BodyId bodyId, Vec2 impulse, Vec2 point, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyLinearImpulseToCenter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyLinearImpulseToCenter(BodyId bodyId, Vec2 impulse, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyMassFromShapes", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyMassFromShapes(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyTorque(BodyId bodyId, float torque, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ComputeAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB BodyComputeAABB(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Disable", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyDisable(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Enable", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyEnable(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableHitEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyEnableHitEvents(BodyId bodyId, byte enableHitEvents);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableSleep", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyEnableSleep(BodyId bodyId, byte enableSleep);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularDamping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetAngularDamping(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetAngularVelocity(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAutomaticMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyGetAutomaticMass(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactCapacity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetContactCapacity(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetContactData(BodyId bodyId, ContactData* contactData, int capacity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetGravityScale", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetGravityScale(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetInertiaTensor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetInertiaTensor(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetJointCount", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetJointCount(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetJoints", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetJoints(BodyId bodyId, JointId* jointArray, int capacity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLinearDamping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetLinearDamping(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLinearVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetLinearVelocity(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalCenterOfMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetLocalCenterOfMass(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetLocalPoint(BodyId bodyId, Vec2 worldPoint);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetLocalVector", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetLocalVector(BodyId bodyId, Vec2 worldVector);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetMass(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetMassData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern MassData BodyGetMassData(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetPosition", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetPosition(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetRotation", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot BodyGetRotation(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetShapeCount", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetShapeCount(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetShapes", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetShapes(BodyId bodyId, ShapeId* shapeArray, int capacity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetSleepThreshold", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetSleepThreshold(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetTransform", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Transform BodyGetTransform(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetType", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyType BodyGetType(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void* BodyGetUserData(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldCenterOfMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetWorldCenterOfMass(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetWorldPoint(BodyId bodyId, Vec2 localPoint);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetWorldVector", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetWorldVector(BodyId bodyId, Vec2 localVector);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsAwake", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyIsAwake(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsBullet", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyIsBullet(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyIsEnabled(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsFixedRotation", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyIsFixedRotation(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsSleepEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyIsSleepEnabled(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte BodyIsValid(BodyId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularDamping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetAngularDamping(BodyId bodyId, float angularDamping);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetAngularVelocity(BodyId bodyId, float angularVelocity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAutomaticMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetAutomaticMass(BodyId bodyId, byte automaticMass);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAwake", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetAwake(BodyId bodyId, byte awake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetBullet", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetBullet(BodyId bodyId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetFixedRotation", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetFixedRotation(BodyId bodyId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetGravityScale", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetGravityScale(BodyId bodyId, float gravityScale);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetLinearDamping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetLinearDamping(BodyId bodyId, float linearDamping);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetLinearVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetLinearVelocity(BodyId bodyId, Vec2 linearVelocity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetMassData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetMassData(BodyId bodyId, MassData massData);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetSleepThreshold", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetSleepThreshold(BodyId bodyId, float sleepVelocity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetTransform", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetTransform(BodyId bodyId, Vec2 position, Rot rotation);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetType", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetType(BodyId bodyId, BodyType type);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetUserData(BodyId bodyId, void* userData);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ChainIsValid(ChainId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetFriction", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ChainSetFriction(ChainId chainId, float friction);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetRestitution", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ChainSetRestitution(ChainId chainId, float restitution);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Clamp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Clamp(Vec2 v, Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ClampFloat", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float ClampFloat(float a, float lower, float upper);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ClampInt", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int ClampInt(int a, int lower, int upper);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCapsuleAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideCapsuleAndCircle(Capsule* capsuleA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCapsules", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideCapsules(Capsule* capsuleA, Transform xfA, Capsule* capsuleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCircles", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideCircles(Circle* circleA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygonAndCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollidePolygonAndCapsule(Polygon* polygonA, Transform xfA, Capsule* capsuleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygonAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollidePolygonAndCircle(Polygon* polygonA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygons", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollidePolygons(Polygon* polygonA, Transform xfA, Polygon* polygonB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSegmentAndCapsule(Segment* segmentA, Transform xfA, Capsule* capsuleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSegmentAndCircle(Segment* segmentA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSegmentAndPolygon(Segment* segmentA, Transform xfA, Polygon* polygonB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSmoothSegmentAndCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSmoothSegmentAndCapsule(SmoothSegment* smoothSegmentA, Transform xfA, Capsule* capsuleB, Transform xfB, DistanceCache* cache);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSmoothSegmentAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSmoothSegmentAndCircle(SmoothSegment* smoothSegmentA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSmoothSegmentAndPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSmoothSegmentAndPolygon(SmoothSegment* smoothSegmentA, Transform xfA, Polygon* polygonB, Transform xfB, DistanceCache* cache);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeAngularVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float ComputeAngularVelocity(Rot q1, Rot q2, float inv_h);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCapsuleAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB ComputeCapsuleAABB(Capsule* shape, Transform transform);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCapsuleMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern MassData ComputeCapsuleMass(Capsule* shape, float density);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCircleAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB ComputeCircleAABB(Circle* shape, Transform transform);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeCircleMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern MassData ComputeCircleMass(Circle* shape, float density);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeHull", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Hull ComputeHull(Vec2* points, int count);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputePolygonAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB ComputePolygonAABB(Polygon* shape, Transform transform);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputePolygonMass", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern MassData ComputePolygonMass(Polygon* shape, float density);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ComputeSegmentAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB ComputeSegmentAABB(Segment* shape, Transform transform);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateBody", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyId CreateBody(WorldId worldId, BodyDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateCapsuleShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeId CreateCapsuleShape(BodyId bodyId, ShapeDef* def, Capsule* capsule);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateChain", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ChainId CreateChain(BodyId bodyId, ChainDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateCircleShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeId CreateCircleShape(BodyId bodyId, ShapeDef* def, Circle* circle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateDistanceJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreateDistanceJoint(WorldId worldId, DistanceJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateMotorJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreateMotorJoint(WorldId worldId, MotorJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateMouseJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreateMouseJoint(WorldId worldId, MouseJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreatePolygonShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeId CreatePolygonShape(BodyId bodyId, ShapeDef* def, Polygon* polygon);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreatePrismaticJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreatePrismaticJoint(WorldId worldId, PrismaticJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateRevoluteJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreateRevoluteJoint(WorldId worldId, RevoluteJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateSegmentShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeId CreateSegmentShape(BodyId bodyId, ShapeDef* def, Segment* segment);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateTimer", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Timer CreateTimer();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateWeldJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreateWeldJoint(WorldId worldId, WeldJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateWheelJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointId CreateWheelJoint(WorldId worldId, WheelJointDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CreateWorld", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern WorldId CreateWorld(WorldDef* def);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Cross", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float Cross(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CrossSV", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 CrossSV(float s, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CrossVS", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 CrossVS(Vec2 v, float s);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultBodyDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyDef DefaultBodyDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultChainDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ChainDef DefaultChainDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultDistanceJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern DistanceJointDef DefaultDistanceJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultFilter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Filter DefaultFilter();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultMotorJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern MotorJointDef DefaultMotorJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultMouseJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern MouseJointDef DefaultMouseJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultPrismaticJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern PrismaticJointDef DefaultPrismaticJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultQueryFilter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern QueryFilter DefaultQueryFilter();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultRevoluteJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern RevoluteJointDef DefaultRevoluteJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultShapeDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeDef DefaultShapeDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultWeldJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern WeldJointDef DefaultWeldJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultWheelJointDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern WheelJointDef DefaultWheelJointDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DefaultWorldDef", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern WorldDef DefaultWorldDef();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyBody", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DestroyBody(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyChain", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DestroyChain(ChainId chainId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyJoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DestroyJoint(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DestroyShape(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyWorld", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DestroyWorld(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Distance", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float Distance(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointEnableLimit(JointId jointId, byte enableLimit);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableMotor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointEnableMotor(JointId jointId, byte enableMotor);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_EnableSpring", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointEnableSpring(JointId jointId, byte enableSpring);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetCurrentLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetCurrentLength(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetLength(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMaxLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetMaxLength(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMaxMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetMaxMotorForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMinLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetMinLength(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetMotorForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetMotorSpeed(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsLimitEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte DistanceJointIsLimitEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsMotorEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte DistanceJointIsMotorEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_IsSpringEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte DistanceJointIsSpringEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetLength(JointId jointId, float length);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetLengthRange", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetLengthRange(JointId jointId, float minLength, float maxLength);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetMaxMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetMaxMotorForce(JointId jointId, float force);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetMotorSpeed(JointId jointId, float motorSpeed);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetSpringHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceSquared", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceSquared(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Dot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float Dot(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Create", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern DynamicTree DynamicTreeCreate();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_CreateProxy", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeCreateProxy(DynamicTree* tree, AABB aabb, uint categoryBits, int userData);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Destroy", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeDestroy(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_DestroyProxy", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeDestroyProxy(DynamicTree* tree, int proxyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_EnlargeProxy", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeEnlargeProxy(DynamicTree* tree, int proxyId, AABB aabb);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB DynamicTreeGetAABB(DynamicTree* tree, int proxyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetAreaRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DynamicTreeGetAreaRatio(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetByteCount", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeGetByteCount(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetHeight", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeGetHeight(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetMaxBalance", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeGetMaxBalance(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetProxyCount", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeGetProxyCount(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_GetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeGetUserData(DynamicTree* tree, int proxyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_MoveProxy", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeMoveProxy(DynamicTree* tree, int proxyId, AABB aabb);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Query", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeQuery(DynamicTree* tree, AABB aabb, uint maskBits, System.IntPtr callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RayCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeRayCast(DynamicTree* tree, RayCastInput* input, uint maskBits, System.IntPtr callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Rebuild", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeRebuild(DynamicTree* tree, byte fullBuild);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RebuildBottomUp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeRebuildBottomUp(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShapeCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeShapeCast(DynamicTree* tree, ShapeCastInput* input, uint maskBits, System.IntPtr callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShiftOrigin", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeShiftOrigin(DynamicTree* tree, Vec2 newOrigin);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Validate", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeValidate(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetByteCount", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int GetByteCount();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetInverse22", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Mat22 GetInverse22(Mat22 A);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthAndNormalize", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 GetLengthAndNormalize(float* length, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthUnitsPerMeter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float GetLengthUnitsPerMeter();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMilliseconds", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float GetMilliseconds(Timer* timer);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMillisecondsAndReset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float GetMillisecondsAndReset(Timer* timer);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetSweepTransform", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Transform GetSweepTransform(Sweep* sweep, float time);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetTicks", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern long GetTicks(Timer* timer);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetVersion", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Version GetVersion();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IntegrateRotation", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot IntegrateRotation(Rot q1, float deltaAngle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulRot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot InvMulRot(Rot q, Rot r);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulTransforms", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Transform InvMulTransforms(Transform A, Transform B);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvRotateVector", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 InvRotateVector(Rot q, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvTransformPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 InvTransformPoint(Transform t, Vec2 p);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsNormalized", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte IsNormalized(Rot q);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte IsValid(float a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IsValidRay", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte IsValidRay(RayCastInput* input);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetBodyA", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyId JointGetBodyA(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetBodyB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyId JointGetBodyB(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetCollideConnected", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte JointGetCollideConnected(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetConstraintForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 JointGetConstraintForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetConstraintTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float JointGetConstraintTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetLocalAnchorA", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 JointGetLocalAnchorA(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetLocalAnchorB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 JointGetLocalAnchorB(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetType", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern JointType JointGetType(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_GetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void* JointGetUserData(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte JointIsValid(JointId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_SetCollideConnected", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void JointSetCollideConnected(JointId jointId, byte shouldCollide);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_SetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void JointSetUserData(JointId jointId, void* userData);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Joint_WakeBodies", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void JointWakeBodies(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LeftPerp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 LeftPerp(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Length", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float Length(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2LengthSquared", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float LengthSquared(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Lerp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Lerp(Vec2 a, Vec2 b, float t);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeBox", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon MakeBox(float hx, float hy);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetBox", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon MakeOffsetBox(float hx, float hy, Vec2 center, float angle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeOffsetPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon MakeOffsetPolygon(Hull* hull, float radius, Transform transform);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakePolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon MakePolygon(Hull* hull, float radius);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeProxy", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern DistanceProxy MakeProxy(Vec2* vertices, int count, float radius);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeRot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot MakeRot(float angle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeRoundedBox", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon MakeRoundedBox(float hx, float hy, float radius);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeSquare", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon MakeSquare(float h);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Max", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Max(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MaxFloat", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MaxFloat(float a, float b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MaxInt", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int MaxInt(int a, int b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Min", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Min(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MinFloat", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MinFloat(float a, float b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MinInt", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int MinInt(int a, int b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetAngularOffset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MotorJointGetAngularOffset(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetCorrectionFactor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MotorJointGetCorrectionFactor(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetLinearOffset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MotorJointGetLinearOffset(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetMaxForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MotorJointGetMaxForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetMaxTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MotorJointGetMaxTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetAngularOffset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MotorJointSetAngularOffset(JointId jointId, float angularOffset);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetCorrectionFactor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MotorJointSetCorrectionFactor(JointId jointId, float correctionFactor);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetLinearOffset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MotorJointSetLinearOffset(JointId jointId, Vec2 linearOffset);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetMaxForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MotorJointSetMaxForce(JointId jointId, float maxForce);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_SetMaxTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MotorJointSetMaxTorque(JointId jointId, float maxTorque);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetMaxForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MouseJointGetMaxForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MouseJointGetSpringDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MouseJointGetSpringHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetTarget", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MouseJointGetTarget(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetMaxForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MouseJointSetMaxForce(JointId jointId, float maxForce);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MouseJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MouseJointSetSpringHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetTarget", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MouseJointSetTarget(JointId jointId, Vec2 target);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Mul", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Mul(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulAdd", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MulAdd(Vec2 a, float s, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulMV", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MulMV(Mat22 A, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulRot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot MulRot(Rot q, Rot r);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulSub", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MulSub(Vec2 a, float s, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulSV", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MulSV(float s, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MulTransforms", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Transform MulTransforms(Transform A, Transform B);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Neg", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Neg(Vec2 a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NLerp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot NLerp(Rot q1, Rot q2, float t);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Normalize", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Normalize(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NormalizeChecked", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 NormalizeChecked(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2NormalizeRot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot NormalizeRot(Rot q);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PointInCapsule(Vec2 point, Capsule* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PointInCircle(Vec2 point, Circle* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PointInPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PointInPolygon(Vec2 point, Polygon* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointEnableLimit(JointId jointId, byte enableLimit);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableMotor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointEnableMotor(JointId jointId, byte enableMotor);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_EnableSpring", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointEnableSpring(JointId jointId, byte enableSpring);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetLowerLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetLowerLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMaxMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetMaxMotorForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetMotorForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetMotorSpeed(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetSpringDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetSpringHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetUpperLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetUpperLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsLimitEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PrismaticJointIsLimitEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsMotorEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PrismaticJointIsMotorEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsSpringEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PrismaticJointIsSpringEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetLimits", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetLimits(JointId jointId, float lower, float upper);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetMaxMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetMaxMotorForce(JointId jointId, float force);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetMotorSpeed(JointId jointId, float motorSpeed);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetSpringHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput RayCastCapsule(RayCastInput* input, Capsule* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput RayCastCircle(RayCastInput* input, Circle* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput RayCastPolygon(RayCastInput* input, Polygon* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RayCastSegment", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput RayCastSegment(RayCastInput* input, Segment* shape, byte oneSided);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RelativeAngle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RelativeAngle(Rot b, Rot a);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointEnableLimit(JointId jointId, byte enableLimit);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableMotor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointEnableMotor(JointId jointId, byte enableMotor);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_EnableSpring", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointEnableSpring(JointId jointId, byte enableSpring);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetAngle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetAngle(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetLowerLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetLowerLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMaxMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetMaxMotorTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetMotorSpeed(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetMotorTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetSpringDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetSpringHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetUpperLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetUpperLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_IsLimitEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte RevoluteJointIsLimitEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_IsMotorEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte RevoluteJointIsMotorEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetLimits", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointSetLimits(JointId jointId, float lower, float upper);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetMaxMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointSetMaxMotorTorque(JointId jointId, float torque);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointSetMotorSpeed(JointId jointId, float motorSpeed);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_SetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void RevoluteJointSetSpringHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RightPerp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 RightPerp(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_GetAngle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RotGetAngle(Rot q);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_GetXAxis", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 RotGetXAxis(Rot q);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_GetYAxis", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 RotGetYAxis(Rot q);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Rot_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte RotIsValid(Rot q);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RotateVector", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 RotateVector(Rot q, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SegmentDistance", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern SegmentDistanceResult SegmentDistance(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetAllocator", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void SetAllocator(System.IntPtr allocFcn, System.IntPtr freeFcn);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetAssertFcn", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void SetAssertFcn(System.IntPtr assertFcn);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetLengthUnitsPerMeter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void SetLengthUnitsPerMeter(float lengthUnits);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreContactEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeAreContactEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreHitEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeAreHitEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_ArePreSolveEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeArePreSolveEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreSensorEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeAreSensorEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableContactEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeEnableContactEvents(ShapeId shapeId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableHitEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeEnableHitEvents(ShapeId shapeId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnablePreSolveEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeEnablePreSolveEvents(ShapeId shapeId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableSensorEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeEnableSensorEvents(ShapeId shapeId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB ShapeGetAABB(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetBody", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyId ShapeGetBody(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Capsule ShapeGetCapsule(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Circle ShapeGetCircle(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetClosestPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 ShapeGetClosestPoint(ShapeId shapeId, Vec2 target);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetContactCapacity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int ShapeGetContactCapacity(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetContactData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int ShapeGetContactData(ShapeId shapeId, ContactData* contactData, int capacity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetDensity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float ShapeGetDensity(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetFilter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Filter ShapeGetFilter(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetFriction", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float ShapeGetFriction(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetParentChain", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ChainId ShapeGetParentChain(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon ShapeGetPolygon(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetRestitution", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float ShapeGetRestitution(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSegment", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Segment ShapeGetSegment(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetSmoothSegment", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern SmoothSegment ShapeGetSmoothSegment(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetType", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeType ShapeGetType(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_GetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void* ShapeGetUserData(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_IsSensor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeIsSensor(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeIsValid(ShapeId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_RayCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput ShapeRayCast(ShapeId shapeId, Vec2 origin, Vec2 translation);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetCapsule(ShapeId shapeId, Capsule* capsule);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetCircle(ShapeId shapeId, Circle* circle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetDensity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetDensity(ShapeId shapeId, float density);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetFilter", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetFilter(ShapeId shapeId, Filter filter);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetFriction", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetFriction(ShapeId shapeId, float friction);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetPolygon(ShapeId shapeId, Polygon* polygon);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetRestitution", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetRestitution(ShapeId shapeId, float restitution);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetSegment", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetSegment(ShapeId shapeId, Segment* segment);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_SetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeSetUserData(ShapeId shapeId, void* userData);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_TestPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeTestPoint(ShapeId shapeId, Vec2 point);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput ShapeCast(ShapeCastPairInput* input);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput ShapeCastCapsule(ShapeCastInput* input, Capsule* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput ShapeCastCircle(ShapeCastInput* input, Circle* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput ShapeCastPolygon(ShapeCastInput* input, Polygon* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeCastSegment", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern CastOutput ShapeCastSegment(ShapeCastInput* input, Segment* shape);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ShapeDistance", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern DistanceOutput ShapeDistance(DistanceCache* cache, DistanceInput* input, Simplex* simplexes, int simplexCapacity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SleepMilliseconds", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void SleepMilliseconds(int milliseconds);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Solve22", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Solve22(Mat22 A, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Sub", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Sub(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TimeOfImpact", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern TOIOutput TimeOfImpact(TOIInput* input);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 TransformPoint(Transform t, Vec2 p);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon TransformPolygon(Transform transform, Polygon* polygon);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2UnwindAngle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float UnwindAngle(float angle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2ValidateHull", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ValidateHull(Hull* hull);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Vec2_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte Vec2IsValid(Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetAngularDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WeldJointGetAngularDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetAngularHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WeldJointGetAngularHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetLinearDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WeldJointGetLinearDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_GetLinearHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WeldJointGetLinearHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetAngularDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WeldJointSetAngularDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetAngularHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WeldJointSetAngularHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetLinearDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WeldJointSetLinearDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WeldJoint_SetLinearHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WeldJointSetLinearHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointEnableLimit(JointId jointId, byte enableLimit);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableMotor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointEnableMotor(JointId jointId, byte enableMotor);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_EnableSpring", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointEnableSpring(JointId jointId, byte enableSpring);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetLowerLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetLowerLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetMaxMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetMaxMotorTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetMotorSpeed(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetMotorTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetSpringDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetSpringHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetUpperLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetUpperLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsLimitEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte WheelJointIsLimitEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsMotorEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte WheelJointIsMotorEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_IsSpringEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte WheelJointIsSpringEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetLimits", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointSetLimits(JointId jointId, float lower, float upper);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetMaxMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointSetMaxMotorTorque(JointId jointId, float torque);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointSetMotorSpeed(JointId jointId, float motorSpeed);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetSpringDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointSetSpringDampingRatio(JointId jointId, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_SetSpringHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WheelJointSetSpringHertz(JointId jointId, float hertz);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldCastCapsule(WorldId worldId, Capsule* capsule, Transform originTransform, Vec2 translation, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldCastCircle(WorldId worldId, Circle* circle, Transform originTransform, Vec2 translation, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldCastPolygon(WorldId worldId, Polygon* polygon, Transform originTransform, Vec2 translation, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastRay", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldCastRay(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CastRayClosest", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern RayResult WorldCastRayClosest(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Draw", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldDraw(WorldId worldId, DebugDraw* draw);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_DumpMemoryStats", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldDumpMemoryStats(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableContinuous", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldEnableContinuous(WorldId worldId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableSleeping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldEnableSleeping(WorldId worldId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableWarmStarting", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldEnableWarmStarting(WorldId worldId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Explode", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldExplode(WorldId worldId, Vec2 position, float radius, float impulse);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetBodyEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyEvents WorldGetBodyEvents(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetContactEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ContactEvents WorldGetContactEvents(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetCounters", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Counters WorldGetCounters(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetGravity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 WorldGetGravity(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetProfile", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Profile WorldGetProfile(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetSensorEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern SensorEvents WorldGetSensorEvents(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte WorldIsValid(WorldId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapAABB(WorldId worldId, AABB aabb, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapCapsule(WorldId worldId, Capsule* capsule, Transform transform, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapCircle(WorldId worldId, Circle* circle, Transform transform, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapPolygon(WorldId worldId, Polygon* polygon, Transform transform, QueryFilter filter, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetContactTuning", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetContactTuning(WorldId worldId, float hertz, float dampingRatio, float pushVelocity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetCustomFilterCallback", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetCustomFilterCallback(WorldId worldId, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetGravity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetGravity(WorldId worldId, Vec2 gravity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetHitEventThreshold", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetHitEventThreshold(WorldId worldId, float value);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetPreSolveCallback", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetPreSolveCallback(WorldId worldId, System.IntPtr fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetRestitutionThreshold", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetRestitutionThreshold(WorldId worldId, float value);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Step", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldStep(WorldId worldId, float timeStep, int subStepCount);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Yield", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void Yield();

        public partial struct AABB
        {
            public Vec2 lowerBound;

            public Vec2 upperBound;
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

            public byte enableSleep;

            public byte isAwake;

            public byte fixedRotation;

            public byte isBullet;

            public byte isEnabled;

            public byte automaticMass;

            public byte allowFastRotation;

            public int internalValue;
        }

        public partial struct BodyEvents
        {
            public BodyMoveEvent* moveEvents;

            public int moveCount;
        }

        public partial struct BodyId
        {
            public int index1;

            public ushort world0;

            public ushort revision;
        }

        public partial struct BodyMoveEvent
        {
            public Transform transform;

            public BodyId bodyId;

            public void* userData;

            public byte fellAsleep;
        }

        public partial struct Capsule
        {
            public Vec2 center1;

            public Vec2 center2;

            public float radius;
        }

        public partial struct CastOutput
        {
            public Vec2 normal;

            public Vec2 point;

            public float fraction;

            public int iterations;

            public byte hit;
        }

        public partial struct ChainDef
        {
            public void* userData;

            public Vec2* points;

            public int count;

            public float friction;

            public float restitution;

            public Filter filter;

            public byte isLoop;

            public int internalValue;
        }

        public partial struct ChainId
        {
            public int index1;

            public ushort world0;

            public ushort revision;
        }

        public partial struct Circle
        {
            public Vec2 center;

            public float radius;
        }

        public partial struct ContactBeginTouchEvent
        {
            public ShapeId shapeIdA;

            public ShapeId shapeIdB;
        }

        public partial struct ContactData
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

        public partial struct ContactEvents
        {
            public ContactBeginTouchEvent* beginEvents;

            public ContactEndTouchEvent* endEvents;

            public ContactHitEvent* hitEvents;

            public int beginCount;

            public int endCount;

            public int hitCount;
        }

        public partial struct ContactHitEvent
        {
            public ShapeId shapeIdA;

            public ShapeId shapeIdB;

            public Vec2 point;

            public Vec2 normal;

            public float approachSpeed;
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

            public fixed int colorCounts[12];
        }

        public partial struct DebugDraw
        {
            public System.IntPtr DrawPolygon; // delegate*<Vec2*, int, HexColor, void*, void>

            public System.IntPtr DrawSolidPolygon; // delegate*<Transform, Vec2*, int, float, HexColor, void*, void>

            public System.IntPtr DrawCircle; // delegate*<Vec2, float, HexColor, void*, void>

            public System.IntPtr DrawSolidCircle; // delegate*<Transform, float, HexColor, void*, void>

            public System.IntPtr DrawCapsule; // delegate*<Vec2, Vec2, float, HexColor, void*, void>

            public System.IntPtr DrawSolidCapsule; // delegate*<Vec2, Vec2, float, HexColor, void*, void>

            public System.IntPtr DrawSegment; // delegate*<Vec2, Vec2, HexColor, void*, void>

            public System.IntPtr DrawTransform; // delegate*<Transform, void*, void>

            public System.IntPtr DrawPoint; // delegate*<Vec2, float, HexColor, void*, void>

            public System.IntPtr DrawString; // delegate*<Vec2, byte*, void*, void>

            public AABB drawingBounds;

            public byte useDrawingBounds;

            public byte drawShapes;

            public byte drawJoints;

            public byte drawJointExtras;

            public byte drawAABBs;

            public byte drawMass;

            public byte drawContacts;

            public byte drawGraphColors;

            public byte drawContactNormals;

            public byte drawContactImpulses;

            public byte drawFrictionImpulses;

            public void* context;
        }

        public partial struct DistanceCache
        {
            public ushort count;

            public fixed byte indexA[3];

            public fixed byte indexB[3];
        }

        public partial struct DistanceInput
        {
            public DistanceProxy proxyA;

            public DistanceProxy proxyB;

            public Transform transformA;

            public Transform transformB;

            public byte useRadii;
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

        public partial struct DistanceOutput
        {
            public Vec2 pointA;

            public Vec2 pointB;

            public float distance;

            public int iterations;

            public int simplexCount;
        }

        public partial struct DistanceProxy
        {
            public points_FixedBuffer points;

            public int count;

            public float radius;

            public partial struct points_FixedBuffer
            {
                public Vec2 Item0;

                public Vec2 Item1;

                public Vec2 Item2;

                public Vec2 Item3;

                public Vec2 Item4;

                public Vec2 Item5;

                public Vec2 Item6;

                public Vec2 Item7;

                public ref Vec2 this[int index] => ref AsSpan()[index];

                [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
                public System.Span<Vec2> AsSpan() => System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref Item0, 8);
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

        public partial struct Filter
        {
            public uint categoryBits;

            public uint maskBits;

            public int groupIndex;
        }

        public partial struct Hull
        {
            public points_FixedBuffer points;

            public int count;

            public partial struct points_FixedBuffer
            {
                public Vec2 Item0;

                public Vec2 Item1;

                public Vec2 Item2;

                public Vec2 Item3;

                public Vec2 Item4;

                public Vec2 Item5;

                public Vec2 Item6;

                public Vec2 Item7;

                public ref Vec2 this[int index] => ref AsSpan()[index];

                [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
                public System.Span<Vec2> AsSpan() => System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref Item0, 8);
            }
        }

        public partial struct JointId
        {
            public int index1;

            public ushort world0;

            public ushort revision;
        }

        public partial struct Manifold
        {
            public points_FixedBuffer points;

            public Vec2 normal;

            public int pointCount;

            public partial struct points_FixedBuffer
            {
                public ManifoldPoint Item0;

                public ManifoldPoint Item1;

                public ref ManifoldPoint this[int index] => ref AsSpan()[index];

                [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
                public System.Span<ManifoldPoint> AsSpan() => System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref Item0, 2);
            }
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

        public partial struct MassData
        {
            public float mass;

            public Vec2 center;

            public float rotationalInertia;
        }

        public partial struct Mat22
        {
            public Vec2 cx;

            public Vec2 cy;
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

        public partial struct Polygon
        {
            public vertices_FixedBuffer vertices;

            public normals_FixedBuffer normals;

            public Vec2 centroid;

            public float radius;

            public int count;

            public partial struct vertices_FixedBuffer
            {
                public Vec2 Item0;

                public Vec2 Item1;

                public Vec2 Item2;

                public Vec2 Item3;

                public Vec2 Item4;

                public Vec2 Item5;

                public Vec2 Item6;

                public Vec2 Item7;

                public ref Vec2 this[int index] => ref AsSpan()[index];

                [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
                public System.Span<Vec2> AsSpan() => System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref Item0, 8);
            }

            public partial struct normals_FixedBuffer
            {
                public Vec2 Item0;

                public Vec2 Item1;

                public Vec2 Item2;

                public Vec2 Item3;

                public Vec2 Item4;

                public Vec2 Item5;

                public Vec2 Item6;

                public Vec2 Item7;

                public ref Vec2 this[int index] => ref AsSpan()[index];

                [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
                public System.Span<Vec2> AsSpan() => System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref Item0, 8);
            }
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

        public partial struct QueryFilter
        {
            public uint categoryBits;

            public uint maskBits;
        }

        public partial struct RayCastInput
        {
            public Vec2 origin;

            public Vec2 translation;

            public float maxFraction;
        }

        public partial struct RayResult
        {
            public ShapeId shapeId;

            public Vec2 point;

            public Vec2 normal;

            public float fraction;

            public byte hit;
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

        public partial struct Rot
        {
            public float c;

            public float s;
        }

        public partial struct Segment
        {
            public Vec2 point1;

            public Vec2 point2;
        }

        public partial struct SegmentDistanceResult
        {
            public Vec2 closest1;

            public Vec2 closest2;

            public float fraction1;

            public float fraction2;

            public float distanceSquared;
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

        public partial struct ShapeCastInput
        {
            public points_FixedBuffer points;

            public int count;

            public float radius;

            public Vec2 translation;

            public float maxFraction;

            public partial struct points_FixedBuffer
            {
                public Vec2 Item0;

                public Vec2 Item1;

                public Vec2 Item2;

                public Vec2 Item3;

                public Vec2 Item4;

                public Vec2 Item5;

                public Vec2 Item6;

                public Vec2 Item7;

                public ref Vec2 this[int index] => ref AsSpan()[index];

                [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
                public System.Span<Vec2> AsSpan() => System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref Item0, 8);
            }
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

        public partial struct ShapeDef
        {
            public void* userData;

            public float friction;

            public float restitution;

            public float density;

            public Filter filter;

            public uint customColor;

            public byte isSensor;

            public byte enableSensorEvents;

            public byte enableContactEvents;

            public byte enableHitEvents;

            public byte enablePreSolveEvents;

            public byte forceContactCreation;

            public int internalValue;
        }

        public partial struct ShapeId
        {
            public int index1;

            public ushort world0;

            public ushort revision;
        }

        public partial struct Simplex
        {
            public SimplexVertex v1;

            public SimplexVertex v2;

            public SimplexVertex v3;

            public int count;
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

        public partial struct SmoothSegment
        {
            public Vec2 ghost1;

            public Segment segment;

            public Vec2 ghost2;

            public int chainId;
        }

        public partial struct Sweep
        {
            public Vec2 localCenter;

            public Vec2 c1;

            public Vec2 c2;

            public Rot q1;

            public Rot q2;
        }

        public partial struct Timer
        {
            public ulong start_sec;

            public ulong start_usec;
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

        public partial struct Transform
        {
            public Vec2 p;

            public Rot q;
        }

        public partial struct TreeNode
        {
            public AABB aabb;

            public uint categoryBits;

            public AnonymousRecord_collision_L608_C2 AnonymousRecord_collision_L608_C2_Field;

            public int child1;

            public int child2;

            public int userData;

            public short height;

            public byte enlarged;

            public fixed byte pad[9];

            public ref int parent => ref AnonymousRecord_collision_L608_C2_Field.parent;

            public ref int next => ref AnonymousRecord_collision_L608_C2_Field.next;

            [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
            public partial struct AnonymousRecord_collision_L608_C2
            {
                [System.Runtime.InteropServices.FieldOffset(0)]
                public int parent;

                [System.Runtime.InteropServices.FieldOffset(0)]
                public int next;
            }
        }

        public partial struct Vec2
        {
            public float x;

            public float y;
        }

        public partial struct Version
        {
            public int major;

            public int minor;

            public int revision;
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

            public byte enableSleep;

            public byte enableContinous;

            public int workerCount;

            public System.IntPtr enqueueTask; // delegate*<System.IntPtr, int, int, void*, void*, void*>

            public System.IntPtr finishTask; // delegate*<void*, void*, void>

            public void* userTaskContext;

            public int internalValue;
        }

        public partial struct WorldId
        {
            public ushort index1;

            public ushort revision;
        }

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

        public const int defaultCategoryBits = 1;

        public const uint defaultMaskBits = 4294967295;

        public const int maxPolygonVertices = 8;

        public const float pi = 3.1415927f;

        public partial class BindgenInternal
        {
            public const string DllImportPath = "box2d";

            static BindgenInternal()
            {
                DllFilePaths = new System.Collections.Generic.List<string>
                {
                    "box2d",
                    "libbox2d",
                    "runtimes/linux-x64/native/libbox2d",
                    "runtimes/linux-arm64/native/libbox2d",
                    "runtimes/osx-x64/native/libbox2d",
                    "runtimes/osx-arm64/native/libbox2d",
                    "runtimes/win-x64/native/box2d",
                    "runtimes/win-arm64/native/box2d"
                };
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Usage", "SYSLIB1054")]
        public partial class BindgenInternal
        {
            public static readonly System.Collections.Generic.List<string> DllFilePaths;

            public static System.IntPtr LibraryHandle = System.IntPtr.Zero;

            public static readonly object Lock = new object ();

            public static bool IsLinux => System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Linux);

            public static bool IsOsx => System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.OSX);

            public static bool IsWindows => System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Windows);

            [System.Runtime.InteropServices.DllImport("libc", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "dlopen")]
            public static extern System.IntPtr LoadLibraryLinux(string? path, int flags);

            [System.Runtime.InteropServices.DllImport("libdl", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "dlopen")]
            public static extern System.IntPtr LoadLibraryOsx(string? path, int flags);

            [System.Runtime.InteropServices.DllImport("kernel32", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "LoadLibrary")]
            public static extern System.IntPtr LoadLibraryWindows(string path);

            [System.Runtime.InteropServices.DllImport("kernel32", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "GetModuleHandle")]
            public static extern System.IntPtr GetModuleHandle(string? name);

            [System.Runtime.InteropServices.DllImport("libc", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "dlsym")]
            public static extern System.IntPtr GetExportLinux(System.IntPtr handle, string name);

            [System.Runtime.InteropServices.DllImport("libdl", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "dlsym")]
            public static extern System.IntPtr GetExportOsx(System.IntPtr handle, string name);

            [System.Runtime.InteropServices.DllImport("kernel32", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "GetProcAddress")]
            public static extern System.IntPtr GetExportWindows(System.IntPtr handle, string name);

            [System.Runtime.InteropServices.DllImport("libc", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "dlerror")]
            public static extern byte* GetLastErrorLinux();

            [System.Runtime.InteropServices.DllImport("libdl", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "dlerror")]
            public static extern byte* GetLastErrorOsx();

            [System.Runtime.InteropServices.DllImport("kernel32", CallingConvention = System.Runtime.InteropServices.CallingConvention.StdCall, CharSet = System.Runtime.InteropServices.CharSet.Ansi, EntryPoint = "GetLastError")]
            public static extern int GetLastErrorWindows();

            public static bool TryLoad(string path, out System.IntPtr handle)
            {
#if NETCOREAPP3_0_OR_GREATER
            return System.Runtime.InteropServices.NativeLibrary.TryLoad(path, System.Reflection.Assembly.GetExecutingAssembly(), null, out handle);
#else
                handle = System.IntPtr.Zero;
                if (IsLinux)
                    handle = LoadLibraryLinux(path, 0x101);
                else if (IsOsx)
                    handle = LoadLibraryOsx(path, 0x101);
                else if (IsWindows)
                    handle = LoadLibraryWindows(path);
                return handle != System.IntPtr.Zero;
#endif
            }

            public static System.IntPtr GetExport(string symbol)
            {
#if NETCOREAPP3_0_OR_GREATER
            return System.Runtime.InteropServices.NativeLibrary.GetExport(LibraryHandle, symbol);
#else
                if (IsLinux)
                {
                    GetLastErrorLinux();
                    System.IntPtr handle = GetExportLinux(LibraryHandle, symbol);
                    if (handle != System.IntPtr.Zero)
                        return handle;
                    byte* errorResult = GetLastErrorLinux();
                    if (errorResult == null)
                        return handle;
                    string errorMessage = System.Runtime.InteropServices.Marshal.PtrToStringAnsi((System.IntPtr)errorResult)!;
                    throw new System.EntryPointNotFoundException(errorMessage);
                }

                if (IsOsx)
                {
                    GetLastErrorOsx();
                    System.IntPtr handle = GetExportOsx(LibraryHandle, symbol);
                    if (handle != System.IntPtr.Zero)
                        return handle;
                    byte* errorResult = GetLastErrorOsx();
                    if (errorResult == null)
                        return handle;
                    string errorMessage = System.Runtime.InteropServices.Marshal.PtrToStringAnsi((System.IntPtr)errorResult)!;
                    throw new System.EntryPointNotFoundException(errorMessage);
                }

                if (IsWindows)
                {
                    System.IntPtr handle = GetExportWindows(LibraryHandle, symbol);
                    if (handle != System.IntPtr.Zero)
                        return handle;
                    int errorCode = GetLastErrorWindows();
                    string errorMessage = new System.ComponentModel.Win32Exception(errorCode).Message;
                    throw new System.EntryPointNotFoundException($"{errorMessage} \"{symbol}\" not found.");
                }

                throw new System.InvalidOperationException($"Failed to export symbol \"{symbol}\" from dll. Platform is not linux, mac, or windows.");
#endif
            }

            public static void ResolveLibrary()
            {
                System.IntPtr handle = default;
#if NETCOREAPP3_0_OR_GREATER
            foreach (string dllFilePath in DllFilePaths)
            {
                if (TryLoad(dllFilePath, out handle))
                    goto Return;
            }
#else
                string fileExtension;
                if (IsLinux)
                    fileExtension = ".so";
                else if (IsOsx)
                    fileExtension = ".dylib";
                else if (IsWindows)
                    fileExtension = ".dll";
                else
                    throw new System.InvalidOperationException("Can't determine native library file extension for the current system.");
                foreach (string dllFilePath in DllFilePaths)
                {
                    string fileName = System.IO.Path.GetFileName(dllFilePath);
                    string parentDir = $"{dllFilePath}/..";
                    string exeDir = System.IO.Path.GetFullPath(System.IO.Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location)!);
                    string searchDir = System.IO.Path.IsPathRooted(dllFilePath) ? System.IO.Path.GetFullPath(parentDir) + "/" : System.IO.Path.GetFullPath($"{exeDir}/{parentDir}") + "/";
                    if (TryLoad($"{searchDir}{fileName}", out handle))
                        goto Return;
                    if (TryLoad($"{searchDir}{fileName}{fileExtension}", out handle))
                        goto Return;
                    if (TryLoad($"{searchDir}lib{fileName}", out handle))
                        goto Return;
                    if (TryLoad($"{searchDir}lib{fileName}{fileExtension}", out handle))
                        goto Return;
                    if (!fileName.StartsWith("lib") || fileName == "lib")
                        continue;
                    string unprefixed = fileName.Substring(4);
                    if (TryLoad($"{searchDir}{unprefixed}", out handle))
                        goto Return;
                    if (TryLoad($"{searchDir}{unprefixed}{fileExtension}", out handle))
                        goto Return;
                }

#endif
#if NET7_0_OR_GREATER
                handle = System.Runtime.InteropServices.NativeLibrary.GetMainProgramHandle();
#else
                if (IsLinux)
                    handle = LoadLibraryLinux(null, 0x101);
                else if (IsOsx)
                    handle = LoadLibraryOsx(null, 0x101);
                else if (IsWindows)
                    handle = GetModuleHandle(null);
#endif
                Return:
                    LibraryHandle = handle;
            }

            public static void* LoadDllSymbol(string variableSymbol, out void* field)
            {
                lock (Lock)
                {
                    if (LibraryHandle == System.IntPtr.Zero)
                        ResolveLibrary();
                    return field = (void*)GetExport(variableSymbol);
                }
            }
        }
    }
}
#pragma warning restore CS9084
#nullable disable
