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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ApplyTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyApplyTorque(BodyId bodyId, float torque, byte wake);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ComputeAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern AABB BodyComputeAABB(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Disable", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyDisable(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Enable", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyEnable(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_EnableSleep", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyEnableSleep(BodyId bodyId, byte enableSleep);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetAngle(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularDamping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetAngularDamping(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetAngularVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetAngularVelocity(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactCapacity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetContactCapacity(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetContactData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int BodyGetContactData(BodyId bodyId, ContactData* contactData, int capacity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetFirstShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeId BodyGetFirstShape(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetGravityScale", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetGravityScale(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetInertiaTensor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float BodyGetInertiaTensor(BodyId bodyId);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetNextShape", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ShapeId BodyGetNextShape(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetPosition", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 BodyGetPosition(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_GetRotation", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot BodyGetRotation(BodyId bodyId);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_ResetMassData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyResetMassData(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularDamping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetAngularDamping(BodyId bodyId, float angularDamping);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetAngularVelocity", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetAngularVelocity(BodyId bodyId, float angularVelocity);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetTransform", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetTransform(BodyId bodyId, Vec2 position, float angle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetType", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetType(BodyId bodyId, BodyType type);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_SetUserData", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodySetUserData(BodyId bodyId, void* userData);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Body_Wake", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void BodyWake(BodyId bodyId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ChainIsValid(ChainId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetFriction", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ChainSetFriction(ChainId chainId, float friction);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Chain_SetRestitution", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ChainSetRestitution(ChainId chainId, float restitution);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Clamp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Clamp(Vec2 v, Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCapsuleAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideCapsuleAndCircle(Capsule* capsuleA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCapsules", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideCapsules(Capsule* capsuleA, Transform xfA, Capsule* capsuleB, Transform xfB, DistanceCache* cache);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideCircles", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideCircles(Circle* circleA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygonAndCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollidePolygonAndCapsule(Polygon* polygonA, Transform xfA, Capsule* capsuleB, Transform xfB, DistanceCache* cache);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygonAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollidePolygonAndCircle(Polygon* polygonA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollidePolygons", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollidePolygons(Polygon* polyA, Transform xfA, Polygon* polyB, Transform xfB, DistanceCache* cache);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSegmentAndCapsule(Segment* segmentA, Transform xfA, Capsule* capsuleB, Transform xfB, DistanceCache* cache);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSegmentAndCircle(Segment* segmentA, Transform xfA, Circle* circleB, Transform xfB);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2CollideSegmentAndPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Manifold CollideSegmentAndPolygon(Segment* segmentA, Transform xfA, Polygon* polygonB, Transform xfB, DistanceCache* cache);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DestroyBodyAndJoints", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DestroyBodyAndJoints(BodyId bodyId);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetConstraintForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetConstraintForce(JointId jointId, float timeStep);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_GetMinLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceJointGetMinLength(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetLength", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetLength(JointId jointId, float length);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetLengthRange", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetLengthRange(JointId jointId, float minLength, float maxLength);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceJoint_SetTuning", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DistanceJointSetTuning(JointId jointId, float hertz, float dampingRatio);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DistanceSquared", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float DistanceSquared(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Dot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float Dot(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Clone", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeClone(DynamicTree* outTree, DynamicTree* inTree);

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
        public static extern void DynamicTreeQuery(DynamicTree* tree, AABB aabb, System.IntPtr* callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_QueryFiltered", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeQueryFiltered(DynamicTree* tree, AABB aabb, uint maskBits, System.IntPtr* callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RayCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeRayCast(DynamicTree* tree, RayCastInput* input, uint maskBits, System.IntPtr* callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Rebuild", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern int DynamicTreeRebuild(DynamicTree* tree, byte fullBuild);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_RebuildBottomUp", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeRebuildBottomUp(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShapeCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeShapeCast(DynamicTree* tree, ShapeCastInput* input, uint maskBits, System.IntPtr* callback, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_ShiftOrigin", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeShiftOrigin(DynamicTree* tree, Vec2 newOrigin);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2DynamicTree_Validate", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void DynamicTreeValidate(DynamicTree* tree);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetByteCount", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern uint GetByteCount();

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetInverse22", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Mat22 GetInverse22(Mat22 A);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetLengthAndNormalize", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 GetLengthAndNormalize(float* length, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMilliseconds", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float GetMilliseconds(Timer* timer);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetMillisecondsAndReset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float GetMillisecondsAndReset(Timer* timer);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetSweepTransform", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Transform GetSweepTransform(Sweep* sweep, float time);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2GetTicks", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern long GetTicks(Timer* timer);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2IntegrateRotation", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot IntegrateRotation(Rot q1, float deltaAngle);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulRot", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Rot InvMulRot(Rot q, Rot r);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvMulTransforms", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Transform InvMulTransforms(Transform A, Transform B);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvRotateVector", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 InvRotateVector(Rot q, Vec2 v);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2InvTransformPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 InvTransformPoint(Transform xf, Vec2 p);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeColor", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Color MakeColor(HexColor hexCode);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MakeColorAlpha", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Color MakeColorAlpha(HexColor hexCode, float alpha);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Min", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Min(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetAngularOffset", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MotorJointGetAngularOffset(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetConstraintForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MotorJointGetConstraintForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MotorJoint_GetConstraintTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MotorJointGetConstraintTorque(JointId jointId);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetDampingRatio", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MouseJointGetDampingRatio(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetHertz", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float MouseJointGetHertz(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_GetTarget", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 MouseJointGetTarget(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetTarget", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MouseJointSetTarget(JointId jointId, Vec2 target);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2MouseJoint_SetTuning", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void MouseJointSetTuning(JointId jointId, float hertz, float dampingRatio);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetConstraintForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 PrismaticJointGetConstraintForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetConstraintTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetConstraintTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetLowerLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetLowerLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMaxMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetMaxMotorForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetMotorForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetMotorSpeed(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_GetUpperLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float PrismaticJointGetUpperLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsLimitEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PrismaticJointIsLimitEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_IsMotorEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte PrismaticJointIsMotorEnabled(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetLimits", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetLimits(JointId jointId, float lower, float upper);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetMaxMotorForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetMaxMotorForce(JointId jointId, float force);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2PrismaticJoint_SetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void PrismaticJointSetMotorSpeed(JointId jointId, float motorSpeed);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetConstraintForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 RevoluteJointGetConstraintForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetConstraintTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetConstraintTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetLowerLimit", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetLowerLimit(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMaxMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetMaxMotorTorque(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMotorSpeed", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetMotorSpeed(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2RevoluteJoint_GetMotorTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float RevoluteJointGetMotorTorque(JointId jointId);

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
        public static extern void SetAllocator(System.IntPtr* allocFcn, System.IntPtr* freeFcn);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SetAssertFcn", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void SetAssertFcn(System.IntPtr* assertFcn);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreContactEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeAreContactEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_ArePreSolveEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeArePreSolveEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_AreSensorEventsEnabled", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte ShapeAreSensorEventsEnabled(ShapeId shapeId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Shape_EnableContactEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void ShapeEnableContactEvents(ShapeId shapeId, byte flag);

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
        public static extern DistanceOutput ShapeDistance(DistanceCache* cache, DistanceInput* input);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2SleepMilliseconds", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void SleepMilliseconds(float milliseconds);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Solve22", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Solve22(Mat22 A, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2Sub", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 Sub(Vec2 a, Vec2 b);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TimeOfImpact", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern TOIOutput TimeOfImpact(TOIInput* input);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPoint", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 TransformPoint(Transform xf, Vec2 p);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2TransformPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Polygon TransformPolygon(Transform transform, Polygon* polygon);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetConstraintForce", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Vec2 WheelJointGetConstraintForce(JointId jointId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2WheelJoint_GetConstraintTorque", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern float WheelJointGetConstraintTorque(JointId jointId);

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

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CapsuleCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldCapsuleCast(WorldId worldId, Capsule* capsule, Transform originTransform, Vec2 translation, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_CircleCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldCircleCast(WorldId worldId, Circle* circle, Transform originTransform, Vec2 translation, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Draw", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldDraw(WorldId worldId, DebugDraw* debugDraw);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableContinuous", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldEnableContinuous(WorldId worldId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableSleeping", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldEnableSleeping(WorldId worldId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_EnableWarmStarting", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldEnableWarmStarting(WorldId worldId, byte flag);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetBodyEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern BodyEvents WorldGetBodyEvents(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetContactEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern ContactEvents WorldGetContactEvents(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetCounters", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Counters WorldGetCounters(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetProfile", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern Profile WorldGetProfile(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_GetSensorEvents", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern SensorEvents WorldGetSensorEvents(WorldId worldId);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_IsValid", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern byte WorldIsValid(WorldId id);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapAABB", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapAABB(WorldId worldId, AABB aabb, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCapsule", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapCapsule(WorldId worldId, Capsule* capsule, Transform transform, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapCircle", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapCircle(WorldId worldId, Circle* circle, Transform transform, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_OverlapPolygon", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldOverlapPolygon(WorldId worldId, Polygon* polygon, Transform transform, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_PolygonCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldPolygonCast(WorldId worldId, Polygon* polygon, Transform originTransform, Vec2 translation, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_RayCast", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldRayCast(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_RayCastClosest", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern RayResult WorldRayCastClosest(WorldId worldId, Vec2 origin, Vec2 translation, QueryFilter filter);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetContactTuning", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetContactTuning(WorldId worldId, float hertz, float dampingRatio, float pushVelocity);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetPreSolveCallback", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetPreSolveCallback(WorldId worldId, System.IntPtr* fcn, void* context);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_SetRestitutionThreshold", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldSetRestitutionThreshold(WorldId worldId, float value);

        [System.Runtime.InteropServices.DllImport(BindgenInternal.DllImportPath, EntryPoint = "b2World_Step", CallingConvention = System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public static extern void WorldStep(WorldId worldId, float timeStep, int subStepCount);

        public partial struct AABB
        {
            public Vec2 lowerBound;

            public Vec2 upperBound;
        }

        public partial struct BodyDef
        {
            public BodyType type;

            public Vec2 position;

            public float angle;

            public Vec2 linearVelocity;

            public float angularVelocity;

            public float linearDamping;

            public float angularDamping;

            public float gravityScale;

            public void* userData;

            public byte enableSleep;

            public byte isAwake;

            public byte fixedRotation;

            public byte isBullet;

            public byte isEnabled;
        }

        public partial struct BodyEvents
        {
            public BodyMoveEvent* moveEvents;

            public int moveCount;
        }

        public partial struct BodyId
        {
            public int index;

            public ushort world;

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
            public Vec2 point1;

            public Vec2 point2;

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
            public Vec2* points;

            public int count;

            public byte isLoop;

            public void* userData;

            public float friction;

            public float restitution;

            public Filter filter;
        }

        public partial struct ChainId
        {
            public int index;

            public short world;

            public ushort revision;
        }

        public partial struct Circle
        {
            public Vec2 point;

            public float radius;
        }

        public partial struct Color
        {
            public float r;

            public float g;

            public float b;

            public float a;
        }

        public partial struct ContactBeginTouchEvent
        {
            public ShapeId shapeIdA;

            public ShapeId shapeIdB;

            public Manifold manifold;
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

            public int beginCount;

            public int endCount;
        }

        public partial struct Counters
        {
            public int islandCount;

            public int bodyCount;

            public int contactCount;

            public int jointCount;

            public int proxyCount;

            public int pairCount;

            public int treeHeight;

            public int stackCapacity;

            public int stackUsed;

            public int byteCount;

            public int taskCount;

            public fixed int colorCounts[13];
        }

        public partial struct DebugDraw
        {
            public System.IntPtr DrawPolygon; // delegate* unmanaged<Vec2*, int, Color, void*, void>

            public System.IntPtr DrawSolidPolygon; // delegate* unmanaged<Vec2*, int, Color, void*, void>

            public System.IntPtr DrawRoundedPolygon; // delegate* unmanaged<Vec2*, int, float, Color, Color, void*, void>

            public System.IntPtr DrawCircle; // delegate* unmanaged<Vec2, float, Color, void*, void>

            public System.IntPtr DrawSolidCircle; // delegate* unmanaged<Vec2, float, Vec2, Color, void*, void>

            public System.IntPtr DrawCapsule; // delegate* unmanaged<Vec2, Vec2, float, Color, void*, void>

            public System.IntPtr DrawSolidCapsule; // delegate* unmanaged<Vec2, Vec2, float, Color, void*, void>

            public System.IntPtr DrawSegment; // delegate* unmanaged<Vec2, Vec2, Color, void*, void>

            public System.IntPtr DrawTransform; // delegate* unmanaged<Transform, void*, void>

            public System.IntPtr DrawPoint; // delegate* unmanaged<Vec2, float, Color, void*, void>

            public System.IntPtr DrawString; // delegate* unmanaged<Vec2, byte*, void*, void>

            public byte drawShapes;

            public byte drawJoints;

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
            public float metric;

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

            public float minLength;

            public float maxLength;

            public float hertz;

            public float dampingRatio;

            public byte collideConnected;

            public void* userData;
        }

        public partial struct DistanceOutput
        {
            public Vec2 pointA;

            public Vec2 pointB;

            public float distance;

            public int iterations;
        }

        public partial struct DistanceProxy
        {
            public vertices_FixedBuffer vertices;

            public int count;

            public float radius;

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
            public int index;

            public ushort world;

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

            public ushort id;

            public byte persisted;
        }

        public partial struct MassData
        {
            public float mass;

            public Vec2 center;

            public float I;
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
        }

        public partial struct MouseJointDef
        {
            public BodyId bodyIdA;

            public BodyId bodyIdB;

            public Vec2 target;

            public float hertz;

            public float dampingRatio;

            public byte collideConnected;

            public void* userData;
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

            public byte enableLimit;

            public float lowerTranslation;

            public float upperTranslation;

            public byte enableMotor;

            public float maxMotorForce;

            public float motorSpeed;

            public byte collideConnected;

            public void* userData;
        }

        public partial struct Profile
        {
            public float step;

            public float pairs;

            public float collide;

            public float solve;

            public float buildIslands;

            public float solveConstraints;

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

            public byte enableLimit;

            public float lowerAngle;

            public float upperAngle;

            public byte enableMotor;

            public float maxMotorTorque;

            public float motorSpeed;

            public float drawSize;

            public byte collideConnected;

            public void* userData;
        }

        public partial struct Rot
        {
            public float s;

            public float c;
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

            public byte isSensor;

            public byte enableSensorEvents;

            public byte enableContactEvents;

            public byte enablePreSolveEvents;
        }

        public partial struct ShapeId
        {
            public int index;

            public ushort world;

            public ushort revision;
        }

        public partial struct SmoothSegment
        {
            public Vec2 ghost1;

            public Segment segment;

            public Vec2 ghost2;

            public int chainIndex;
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

            public AnonymousRecord_dynamic_tree_L27_C2 AnonymousRecord_dynamic_tree_L27_C2_Field;

            public int child1;

            public int child2;

            public int userData;

            public short height;

            public byte enlarged;

            public fixed byte pad[9];

            public ref int parent => ref AnonymousRecord_dynamic_tree_L27_C2_Field.parent;

            public ref int next => ref AnonymousRecord_dynamic_tree_L27_C2_Field.next;

            [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
            public partial struct AnonymousRecord_dynamic_tree_L27_C2
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
        }

        public partial struct WheelJointDef
        {
            public BodyId bodyIdA;

            public BodyId bodyIdB;

            public Vec2 localAnchorA;

            public Vec2 localAnchorB;

            public Vec2 localAxisA;

            public byte enableLimit;

            public float lowerTranslation;

            public float upperTranslation;

            public byte enableMotor;

            public float maxMotorTorque;

            public float motorSpeed;

            public float hertz;

            public float dampingRatio;

            public byte collideConnected;

            public void* userData;
        }

        public partial struct WorldDef
        {
            public Vec2 gravity;

            public float restitutionThreshold;

            public float contactPushoutVelocity;

            public float contactHertz;

            public float contactDampingRatio;

            public float jointHertz;

            public float jointDampingRatio;

            public byte enableSleep;

            public byte enableContinous;

            public int bodyCapacity;

            public int shapeCapacity;

            public int contactCapacity;

            public int jointCapacity;

            public int stackAllocatorCapacity;

            public uint workerCount;

            public System.IntPtr* enqueueTask; // delegate* unmanaged<System.IntPtr, int, int, void*, void*, void*>

            public System.IntPtr* finishTask; // delegate* unmanaged<void*, void*, void>

            public void* userTaskContext;
        }

        public partial struct WorldId
        {
            public ushort index;

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
            colorAntiqueWhite1 = 16773083,
            colorAntiqueWhite2 = 15654860,
            colorAntiqueWhite3 = 13484208,
            colorAntiqueWhite4 = 9143160,
            colorAqua = 65535,
            colorAquamarine = 8388564,
            colorAquamarine1 = 8388564,
            colorAquamarine2 = 7794374,
            colorAquamarine3 = 6737322,
            colorAquamarine4 = 4557684,
            colorAzure = 15794175,
            colorAzure1 = 15794175,
            colorAzure2 = 14741230,
            colorAzure3 = 12701133,
            colorAzure4 = 8620939,
            colorBeige = 16119260,
            colorBisque = 16770244,
            colorBisque1 = 16770244,
            colorBisque2 = 15652279,
            colorBisque3 = 13481886,
            colorBisque4 = 9141611,
            colorBlack = 0,
            colorBlanchedAlmond = 16772045,
            colorBlue = 255,
            colorBlue1 = 255,
            colorBlue2 = 238,
            colorBlue3 = 205,
            colorBlue4 = 139,
            colorBlueViolet = 9055202,
            colorBrown = 10824234,
            colorBrown1 = 16728128,
            colorBrown2 = 15612731,
            colorBrown3 = 13447987,
            colorBrown4 = 9118499,
            colorBurlywood = 14596231,
            colorBurlywood1 = 16765851,
            colorBurlywood2 = 15648145,
            colorBurlywood3 = 13478525,
            colorBurlywood4 = 9139029,
            colorCadetBlue = 6266528,
            colorCadetBlue1 = 10024447,
            colorCadetBlue2 = 9364974,
            colorCadetBlue3 = 8046029,
            colorCadetBlue4 = 5473931,
            colorChartreuse = 8388352,
            colorChartreuse1 = 8388352,
            colorChartreuse2 = 7794176,
            colorChartreuse3 = 6737152,
            colorChartreuse4 = 4557568,
            colorChocolate = 13789470,
            colorChocolate1 = 16744228,
            colorChocolate2 = 15627809,
            colorChocolate3 = 13461021,
            colorChocolate4 = 9127187,
            colorCoral = 16744272,
            colorCoral1 = 16740950,
            colorCoral2 = 15624784,
            colorCoral3 = 13458245,
            colorCoral4 = 9125423,
            colorCornflowerBlue = 6591981,
            colorCornsilk = 16775388,
            colorCornsilk1 = 16775388,
            colorCornsilk2 = 15657165,
            colorCornsilk3 = 13486257,
            colorCornsilk4 = 9144440,
            colorCrimson = 14423100,
            colorCyan = 65535,
            colorCyan1 = 65535,
            colorCyan2 = 61166,
            colorCyan3 = 52685,
            colorCyan4 = 35723,
            colorDarkBlue = 139,
            colorDarkCyan = 35723,
            colorDarkGoldenrod = 12092939,
            colorDarkGoldenrod1 = 16759055,
            colorDarkGoldenrod2 = 15641870,
            colorDarkGoldenrod3 = 13473036,
            colorDarkGoldenrod4 = 9135368,
            colorDarkGray = 11119017,
            colorDarkGreen = 25600,
            colorDarkKhaki = 12433259,
            colorDarkMagenta = 9109643,
            colorDarkOliveGreen = 5597999,
            colorDarkOliveGreen1 = 13303664,
            colorDarkOliveGreen2 = 12381800,
            colorDarkOliveGreen3 = 10669402,
            colorDarkOliveGreen4 = 7244605,
            colorDarkOrange = 16747520,
            colorDarkOrange1 = 16744192,
            colorDarkOrange2 = 15627776,
            colorDarkOrange3 = 13460992,
            colorDarkOrange4 = 9127168,
            colorDarkOrchid = 10040012,
            colorDarkOrchid1 = 12533503,
            colorDarkOrchid2 = 11680494,
            colorDarkOrchid3 = 10105549,
            colorDarkOrchid4 = 6824587,
            colorDarkRed = 9109504,
            colorDarkSalmon = 15308410,
            colorDarkSeaGreen = 9419919,
            colorDarkSeaGreen1 = 12713921,
            colorDarkSeaGreen2 = 11857588,
            colorDarkSeaGreen3 = 10210715,
            colorDarkSeaGreen4 = 6916969,
            colorDarkSlateBlue = 4734347,
            colorDarkSlateGray = 3100495,
            colorDarkSlateGray1 = 9961471,
            colorDarkSlateGray2 = 9301742,
            colorDarkSlateGray3 = 7982541,
            colorDarkSlateGray4 = 5409675,
            colorDarkTurquoise = 52945,
            colorDarkViolet = 9699539,
            colorDeepPink = 16716947,
            colorDeepPink1 = 16716947,
            colorDeepPink2 = 15602313,
            colorDeepPink3 = 13439094,
            colorDeepPink4 = 9112144,
            colorDeepSkyBlue = 49151,
            colorDeepSkyBlue1 = 49151,
            colorDeepSkyBlue2 = 45806,
            colorDeepSkyBlue3 = 39629,
            colorDeepSkyBlue4 = 26763,
            colorDimGray = 6908265,
            colorDodgerBlue = 2003199,
            colorDodgerBlue1 = 2003199,
            colorDodgerBlue2 = 1869550,
            colorDodgerBlue3 = 1602765,
            colorDodgerBlue4 = 1068683,
            colorFirebrick = 11674146,
            colorFirebrick1 = 16724016,
            colorFirebrick2 = 15608876,
            colorFirebrick3 = 13444646,
            colorFirebrick4 = 9116186,
            colorFloralWhite = 16775920,
            colorForestGreen = 2263842,
            colorFuchsia = 16711935,
            colorGainsboro = 14474460,
            colorGhostWhite = 16316671,
            colorGold = 16766720,
            colorGold1 = 16766720,
            colorGold2 = 15649024,
            colorGold3 = 13479168,
            colorGold4 = 9139456,
            colorGoldenrod = 14329120,
            colorGoldenrod1 = 16761125,
            colorGoldenrod2 = 15643682,
            colorGoldenrod3 = 13474589,
            colorGoldenrod4 = 9136404,
            colorGray = 12500670,
            colorGray0 = 0,
            colorGray1 = 197379,
            colorGray10 = 1710618,
            colorGray100 = 16777215,
            colorGray11 = 1842204,
            colorGray12 = 2039583,
            colorGray13 = 2171169,
            colorGray14 = 2368548,
            colorGray15 = 2500134,
            colorGray16 = 2697513,
            colorGray17 = 2829099,
            colorGray18 = 3026478,
            colorGray19 = 3158064,
            colorGray2 = 328965,
            colorGray20 = 3355443,
            colorGray21 = 3552822,
            colorGray22 = 3684408,
            colorGray23 = 3881787,
            colorGray24 = 4013373,
            colorGray25 = 4210752,
            colorGray26 = 4342338,
            colorGray27 = 4539717,
            colorGray28 = 4671303,
            colorGray29 = 4868682,
            colorGray3 = 526344,
            colorGray30 = 5066061,
            colorGray31 = 5197647,
            colorGray32 = 5395026,
            colorGray33 = 5526612,
            colorGray34 = 5723991,
            colorGray35 = 5855577,
            colorGray36 = 6052956,
            colorGray37 = 6184542,
            colorGray38 = 6381921,
            colorGray39 = 6513507,
            colorGray4 = 657930,
            colorGray40 = 6710886,
            colorGray41 = 6908265,
            colorGray42 = 7039851,
            colorGray43 = 7237230,
            colorGray44 = 7368816,
            colorGray45 = 7566195,
            colorGray46 = 7697781,
            colorGray47 = 7895160,
            colorGray48 = 8026746,
            colorGray49 = 8224125,
            colorGray5 = 855309,
            colorGray50 = 8355711,
            colorGray51 = 8553090,
            colorGray52 = 8750469,
            colorGray53 = 8882055,
            colorGray54 = 9079434,
            colorGray55 = 9211020,
            colorGray56 = 9408399,
            colorGray57 = 9539985,
            colorGray58 = 9737364,
            colorGray59 = 9868950,
            colorGray6 = 986895,
            colorGray60 = 10066329,
            colorGray61 = 10263708,
            colorGray62 = 10395294,
            colorGray63 = 10592673,
            colorGray64 = 10724259,
            colorGray65 = 10921638,
            colorGray66 = 11053224,
            colorGray67 = 11250603,
            colorGray68 = 11382189,
            colorGray69 = 11579568,
            colorGray7 = 1184274,
            colorGray70 = 11776947,
            colorGray71 = 11908533,
            colorGray72 = 12105912,
            colorGray73 = 12237498,
            colorGray74 = 12434877,
            colorGray75 = 12566463,
            colorGray76 = 12763842,
            colorGray77 = 12895428,
            colorGray78 = 13092807,
            colorGray79 = 13224393,
            colorGray8 = 1315860,
            colorGray80 = 13421772,
            colorGray81 = 13619151,
            colorGray82 = 13750737,
            colorGray83 = 13948116,
            colorGray84 = 14079702,
            colorGray85 = 14277081,
            colorGray86 = 14408667,
            colorGray87 = 14606046,
            colorGray88 = 14737632,
            colorGray89 = 14935011,
            colorGray9 = 1513239,
            colorGray90 = 15066597,
            colorGray91 = 15263976,
            colorGray92 = 15461355,
            colorGray93 = 15592941,
            colorGray94 = 15790320,
            colorGray95 = 15921906,
            colorGray96 = 16119285,
            colorGray97 = 16250871,
            colorGray98 = 16448250,
            colorGray99 = 16579836,
            colorGreen = 65280,
            colorGreen1 = 65280,
            colorGreen2 = 60928,
            colorGreen3 = 52480,
            colorGreen4 = 35584,
            colorGreenYellow = 11403055,
            colorHoneydew = 15794160,
            colorHoneydew1 = 15794160,
            colorHoneydew2 = 14741216,
            colorHoneydew3 = 12701121,
            colorHoneydew4 = 8620931,
            colorHotPink = 16738740,
            colorHotPink1 = 16740020,
            colorHotPink2 = 15624871,
            colorHotPink3 = 13459600,
            colorHotPink4 = 9124450,
            colorIndianRed = 13458524,
            colorIndianRed1 = 16738922,
            colorIndianRed2 = 15623011,
            colorIndianRed3 = 13456725,
            colorIndianRed4 = 9124410,
            colorIndigo = 4915330,
            colorIvory = 16777200,
            colorIvory1 = 16777200,
            colorIvory2 = 15658720,
            colorIvory3 = 13487553,
            colorIvory4 = 9145219,
            colorKhaki = 15787660,
            colorKhaki1 = 16774799,
            colorKhaki2 = 15656581,
            colorKhaki3 = 13485683,
            colorKhaki4 = 9143886,
            colorLavender = 15132410,
            colorLavenderBlush = 16773365,
            colorLavenderBlush1 = 16773365,
            colorLavenderBlush2 = 15655141,
            colorLavenderBlush3 = 13484485,
            colorLavenderBlush4 = 9143174,
            colorLawnGreen = 8190976,
            colorLemonChiffon = 16775885,
            colorLemonChiffon1 = 16775885,
            colorLemonChiffon2 = 15657407,
            colorLemonChiffon3 = 13486501,
            colorLemonChiffon4 = 9144688,
            colorLightBlue = 11393254,
            colorLightBlue1 = 12578815,
            colorLightBlue2 = 11722734,
            colorLightBlue3 = 10141901,
            colorLightBlue4 = 6849419,
            colorLightCoral = 15761536,
            colorLightCyan = 14745599,
            colorLightCyan1 = 14745599,
            colorLightCyan2 = 13758190,
            colorLightCyan3 = 11849165,
            colorLightCyan4 = 8031115,
            colorLightGoldenrod = 15654274,
            colorLightGoldenrod1 = 16772235,
            colorLightGoldenrod2 = 15654018,
            colorLightGoldenrod3 = 13483632,
            colorLightGoldenrod4 = 9142604,
            colorLightGoldenrodYellow = 16448210,
            colorLightGray = 13882323,
            colorLightGreen = 9498256,
            colorLightPink = 16758465,
            colorLightPink1 = 16756409,
            colorLightPink2 = 15639213,
            colorLightPink3 = 13470869,
            colorLightPink4 = 9133925,
            colorLightSalmon = 16752762,
            colorLightSalmon1 = 16752762,
            colorLightSalmon2 = 15635826,
            colorLightSalmon3 = 13468002,
            colorLightSalmon4 = 9131842,
            colorLightSeaGreen = 2142890,
            colorLightSkyBlue = 8900346,
            colorLightSkyBlue1 = 11592447,
            colorLightSkyBlue2 = 10802158,
            colorLightSkyBlue3 = 9287373,
            colorLightSkyBlue4 = 6323083,
            colorLightSlateBlue = 8679679,
            colorLightSlateGray = 7833753,
            colorLightSteelBlue = 11584734,
            colorLightSteelBlue1 = 13296127,
            colorLightSteelBlue2 = 12374766,
            colorLightSteelBlue3 = 10663373,
            colorLightSteelBlue4 = 7240587,
            colorLightYellow = 16777184,
            colorLightYellow1 = 16777184,
            colorLightYellow2 = 15658705,
            colorLightYellow3 = 13487540,
            colorLightYellow4 = 9145210,
            colorLime = 65280,
            colorLimeGreen = 3329330,
            colorLinen = 16445670,
            colorMagenta = 16711935,
            colorMagenta1 = 16711935,
            colorMagenta2 = 15597806,
            colorMagenta3 = 13435085,
            colorMagenta4 = 9109643,
            colorMaroon = 11546720,
            colorMaroon1 = 16725171,
            colorMaroon2 = 15610023,
            colorMaroon3 = 13445520,
            colorMaroon4 = 9116770,
            colorMediumAquamarine = 6737322,
            colorMediumBlue = 205,
            colorMediumOrchid = 12211667,
            colorMediumOrchid1 = 14706431,
            colorMediumOrchid2 = 13721582,
            colorMediumOrchid3 = 11817677,
            colorMediumOrchid4 = 8009611,
            colorMediumPurple = 9662683,
            colorMediumPurple1 = 11240191,
            colorMediumPurple2 = 10451438,
            colorMediumPurple3 = 9005261,
            colorMediumPurple4 = 6113163,
            colorMediumSeaGreen = 3978097,
            colorMediumSlateBlue = 8087790,
            colorMediumSpringGreen = 64154,
            colorMediumTurquoise = 4772300,
            colorMediumVioletRed = 13047173,
            colorMidnightBlue = 1644912,
            colorMintCream = 16121850,
            colorMistyRose = 16770273,
            colorMistyRose1 = 16770273,
            colorMistyRose2 = 15652306,
            colorMistyRose3 = 13481909,
            colorMistyRose4 = 9141627,
            colorMoccasin = 16770229,
            colorNavajoWhite = 16768685,
            colorNavajoWhite1 = 16768685,
            colorNavajoWhite2 = 15650721,
            colorNavajoWhite3 = 13480843,
            colorNavajoWhite4 = 9140574,
            colorNavy = 128,
            colorNavyBlue = 128,
            colorOldLace = 16643558,
            colorOlive = 8421376,
            colorOliveDrab = 7048739,
            colorOliveDrab1 = 12648254,
            colorOliveDrab2 = 11791930,
            colorOliveDrab3 = 10145074,
            colorOliveDrab4 = 6916898,
            colorOrange = 16753920,
            colorOrange1 = 16753920,
            colorOrange2 = 15636992,
            colorOrange3 = 13468928,
            colorOrange4 = 9132544,
            colorOrangeRed = 16729344,
            colorOrangeRed1 = 16729344,
            colorOrangeRed2 = 15613952,
            colorOrangeRed3 = 13448960,
            colorOrangeRed4 = 9118976,
            colorOrchid = 14315734,
            colorOrchid1 = 16745466,
            colorOrchid2 = 15629033,
            colorOrchid3 = 13461961,
            colorOrchid4 = 9127817,
            colorPaleGoldenrod = 15657130,
            colorPaleGreen = 10025880,
            colorPaleGreen1 = 10157978,
            colorPaleGreen2 = 9498256,
            colorPaleGreen3 = 8179068,
            colorPaleGreen4 = 5540692,
            colorPaleTurquoise = 11529966,
            colorPaleTurquoise1 = 12320767,
            colorPaleTurquoise2 = 11464430,
            colorPaleTurquoise3 = 9883085,
            colorPaleTurquoise4 = 6720395,
            colorPaleVioletRed = 14381203,
            colorPaleVioletRed1 = 16745131,
            colorPaleVioletRed2 = 15628703,
            colorPaleVioletRed3 = 13461641,
            colorPaleVioletRed4 = 9127773,
            colorPapayaWhip = 16773077,
            colorPeachPuff = 16767673,
            colorPeachPuff1 = 16767673,
            colorPeachPuff2 = 15649709,
            colorPeachPuff3 = 13479829,
            colorPeachPuff4 = 9140069,
            colorPeru = 13468991,
            colorPink = 16761035,
            colorPink1 = 16758213,
            colorPink2 = 15641016,
            colorPink3 = 13472158,
            colorPink4 = 9134956,
            colorPlum = 14524637,
            colorPlum1 = 16759807,
            colorPlum2 = 15642350,
            colorPlum3 = 13473485,
            colorPlum4 = 9135755,
            colorPowderBlue = 11591910,
            colorPurple = 10494192,
            colorPurple1 = 10170623,
            colorPurple2 = 9514222,
            colorPurple3 = 8201933,
            colorPurple4 = 5577355,
            colorRebeccaPurple = 6697881,
            colorRed = 16711680,
            colorRed1 = 16711680,
            colorRed2 = 15597568,
            colorRed3 = 13434880,
            colorRed4 = 9109504,
            colorRosyBrown = 12357519,
            colorRosyBrown1 = 16761281,
            colorRosyBrown2 = 15643828,
            colorRosyBrown3 = 13474715,
            colorRosyBrown4 = 9136489,
            colorRoyalBlue = 4286945,
            colorRoyalBlue1 = 4749055,
            colorRoyalBlue2 = 4419310,
            colorRoyalBlue3 = 3825613,
            colorRoyalBlue4 = 2572427,
            colorSaddleBrown = 9127187,
            colorSalmon = 16416882,
            colorSalmon1 = 16747625,
            colorSalmon2 = 15630946,
            colorSalmon3 = 13463636,
            colorSalmon4 = 9129017,
            colorSandyBrown = 16032864,
            colorSeaGreen = 3050327,
            colorSeaGreen1 = 5570463,
            colorSeaGreen2 = 5172884,
            colorSeaGreen3 = 4443520,
            colorSeaGreen4 = 3050327,
            colorSeashell = 16774638,
            colorSeashell1 = 16774638,
            colorSeashell2 = 15656414,
            colorSeashell3 = 13485503,
            colorSeashell4 = 9143938,
            colorSienna = 10506797,
            colorSienna1 = 16745031,
            colorSienna2 = 15628610,
            colorSienna3 = 13461561,
            colorSienna4 = 9127718,
            colorSilver = 12632256,
            colorSkyBlue = 8900331,
            colorSkyBlue1 = 8900351,
            colorSkyBlue2 = 8306926,
            colorSkyBlue3 = 7120589,
            colorSkyBlue4 = 4878475,
            colorSlateBlue = 6970061,
            colorSlateBlue1 = 8613887,
            colorSlateBlue2 = 8021998,
            colorSlateBlue3 = 6904269,
            colorSlateBlue4 = 4668555,
            colorSlateGray = 7372944,
            colorSlateGray1 = 13034239,
            colorSlateGray2 = 12178414,
            colorSlateGray3 = 10467021,
            colorSlateGray4 = 7109515,
            colorSnow = 16775930,
            colorSnow1 = 16775930,
            colorSnow2 = 15657449,
            colorSnow3 = 13486537,
            colorSnow4 = 9144713,
            colorSpringGreen = 65407,
            colorSpringGreen1 = 65407,
            colorSpringGreen2 = 61046,
            colorSpringGreen3 = 52582,
            colorSpringGreen4 = 35653,
            colorSteelBlue = 4620980,
            colorSteelBlue1 = 6535423,
            colorSteelBlue2 = 6073582,
            colorSteelBlue3 = 5215437,
            colorSteelBlue4 = 3564683,
            colorTan = 13808780,
            colorTan1 = 16753999,
            colorTan2 = 15637065,
            colorTan3 = 13468991,
            colorTan4 = 9132587,
            colorTeal = 32896,
            colorThistle = 14204888,
            colorThistle1 = 16769535,
            colorThistle2 = 15651566,
            colorThistle3 = 13481421,
            colorThistle4 = 9141131,
            colorTomato = 16737095,
            colorTomato1 = 16737095,
            colorTomato2 = 15621186,
            colorTomato3 = 13455161,
            colorTomato4 = 9123366,
            colorTurquoise = 4251856,
            colorTurquoise1 = 62975,
            colorTurquoise2 = 58862,
            colorTurquoise3 = 50637,
            colorTurquoise4 = 34443,
            colorViolet = 15631086,
            colorVioletRed = 13639824,
            colorVioletRed1 = 16727702,
            colorVioletRed2 = 15612556,
            colorVioletRed3 = 13447800,
            colorVioletRed4 = 9118290,
            colorWebGray = 8421504,
            colorWebGreen = 32768,
            colorWebMaroon = 8388608,
            colorWebPurple = 8388736,
            colorWheat = 16113331,
            colorWheat1 = 16771002,
            colorWheat2 = 15653038,
            colorWheat3 = 13482646,
            colorWheat4 = 9141862,
            colorWhite = 16777215,
            colorWhiteSmoke = 16119285,
            colorX11Gray = 12500670,
            colorX11Green = 65280,
            colorX11Maroon = 11546720,
            colorX11Purple = 10494192,
            colorYellow = 16776960,
            colorYellow1 = 16776960,
            colorYellow2 = 15658496,
            colorYellow3 = 13487360,
            colorYellow4 = 9145088,
            colorYellowGreen = 10145074
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

        public const HexColor colorAntiqueWhite1 = HexColor.colorAntiqueWhite1;

        public const HexColor colorAntiqueWhite2 = HexColor.colorAntiqueWhite2;

        public const HexColor colorAntiqueWhite3 = HexColor.colorAntiqueWhite3;

        public const HexColor colorAntiqueWhite4 = HexColor.colorAntiqueWhite4;

        public const HexColor colorAqua = HexColor.colorAqua;

        public const HexColor colorAquamarine = HexColor.colorAquamarine;

        public const HexColor colorAquamarine1 = HexColor.colorAquamarine1;

        public const HexColor colorAquamarine2 = HexColor.colorAquamarine2;

        public const HexColor colorAquamarine3 = HexColor.colorAquamarine3;

        public const HexColor colorAquamarine4 = HexColor.colorAquamarine4;

        public const HexColor colorAzure = HexColor.colorAzure;

        public const HexColor colorAzure1 = HexColor.colorAzure1;

        public const HexColor colorAzure2 = HexColor.colorAzure2;

        public const HexColor colorAzure3 = HexColor.colorAzure3;

        public const HexColor colorAzure4 = HexColor.colorAzure4;

        public const HexColor colorBeige = HexColor.colorBeige;

        public const HexColor colorBisque = HexColor.colorBisque;

        public const HexColor colorBisque1 = HexColor.colorBisque1;

        public const HexColor colorBisque2 = HexColor.colorBisque2;

        public const HexColor colorBisque3 = HexColor.colorBisque3;

        public const HexColor colorBisque4 = HexColor.colorBisque4;

        public const HexColor colorBlack = HexColor.colorBlack;

        public const HexColor colorBlanchedAlmond = HexColor.colorBlanchedAlmond;

        public const HexColor colorBlue = HexColor.colorBlue;

        public const HexColor colorBlue1 = HexColor.colorBlue1;

        public const HexColor colorBlue2 = HexColor.colorBlue2;

        public const HexColor colorBlue3 = HexColor.colorBlue3;

        public const HexColor colorBlue4 = HexColor.colorBlue4;

        public const HexColor colorBlueViolet = HexColor.colorBlueViolet;

        public const HexColor colorBrown = HexColor.colorBrown;

        public const HexColor colorBrown1 = HexColor.colorBrown1;

        public const HexColor colorBrown2 = HexColor.colorBrown2;

        public const HexColor colorBrown3 = HexColor.colorBrown3;

        public const HexColor colorBrown4 = HexColor.colorBrown4;

        public const HexColor colorBurlywood = HexColor.colorBurlywood;

        public const HexColor colorBurlywood1 = HexColor.colorBurlywood1;

        public const HexColor colorBurlywood2 = HexColor.colorBurlywood2;

        public const HexColor colorBurlywood3 = HexColor.colorBurlywood3;

        public const HexColor colorBurlywood4 = HexColor.colorBurlywood4;

        public const HexColor colorCadetBlue = HexColor.colorCadetBlue;

        public const HexColor colorCadetBlue1 = HexColor.colorCadetBlue1;

        public const HexColor colorCadetBlue2 = HexColor.colorCadetBlue2;

        public const HexColor colorCadetBlue3 = HexColor.colorCadetBlue3;

        public const HexColor colorCadetBlue4 = HexColor.colorCadetBlue4;

        public const HexColor colorChartreuse = HexColor.colorChartreuse;

        public const HexColor colorChartreuse1 = HexColor.colorChartreuse1;

        public const HexColor colorChartreuse2 = HexColor.colorChartreuse2;

        public const HexColor colorChartreuse3 = HexColor.colorChartreuse3;

        public const HexColor colorChartreuse4 = HexColor.colorChartreuse4;

        public const HexColor colorChocolate = HexColor.colorChocolate;

        public const HexColor colorChocolate1 = HexColor.colorChocolate1;

        public const HexColor colorChocolate2 = HexColor.colorChocolate2;

        public const HexColor colorChocolate3 = HexColor.colorChocolate3;

        public const HexColor colorChocolate4 = HexColor.colorChocolate4;

        public const HexColor colorCoral = HexColor.colorCoral;

        public const HexColor colorCoral1 = HexColor.colorCoral1;

        public const HexColor colorCoral2 = HexColor.colorCoral2;

        public const HexColor colorCoral3 = HexColor.colorCoral3;

        public const HexColor colorCoral4 = HexColor.colorCoral4;

        public const HexColor colorCornflowerBlue = HexColor.colorCornflowerBlue;

        public const HexColor colorCornsilk = HexColor.colorCornsilk;

        public const HexColor colorCornsilk1 = HexColor.colorCornsilk1;

        public const HexColor colorCornsilk2 = HexColor.colorCornsilk2;

        public const HexColor colorCornsilk3 = HexColor.colorCornsilk3;

        public const HexColor colorCornsilk4 = HexColor.colorCornsilk4;

        public const HexColor colorCrimson = HexColor.colorCrimson;

        public const HexColor colorCyan = HexColor.colorCyan;

        public const HexColor colorCyan1 = HexColor.colorCyan1;

        public const HexColor colorCyan2 = HexColor.colorCyan2;

        public const HexColor colorCyan3 = HexColor.colorCyan3;

        public const HexColor colorCyan4 = HexColor.colorCyan4;

        public const HexColor colorDarkBlue = HexColor.colorDarkBlue;

        public const HexColor colorDarkCyan = HexColor.colorDarkCyan;

        public const HexColor colorDarkGoldenrod = HexColor.colorDarkGoldenrod;

        public const HexColor colorDarkGoldenrod1 = HexColor.colorDarkGoldenrod1;

        public const HexColor colorDarkGoldenrod2 = HexColor.colorDarkGoldenrod2;

        public const HexColor colorDarkGoldenrod3 = HexColor.colorDarkGoldenrod3;

        public const HexColor colorDarkGoldenrod4 = HexColor.colorDarkGoldenrod4;

        public const HexColor colorDarkGray = HexColor.colorDarkGray;

        public const HexColor colorDarkGreen = HexColor.colorDarkGreen;

        public const HexColor colorDarkKhaki = HexColor.colorDarkKhaki;

        public const HexColor colorDarkMagenta = HexColor.colorDarkMagenta;

        public const HexColor colorDarkOliveGreen = HexColor.colorDarkOliveGreen;

        public const HexColor colorDarkOliveGreen1 = HexColor.colorDarkOliveGreen1;

        public const HexColor colorDarkOliveGreen2 = HexColor.colorDarkOliveGreen2;

        public const HexColor colorDarkOliveGreen3 = HexColor.colorDarkOliveGreen3;

        public const HexColor colorDarkOliveGreen4 = HexColor.colorDarkOliveGreen4;

        public const HexColor colorDarkOrange = HexColor.colorDarkOrange;

        public const HexColor colorDarkOrange1 = HexColor.colorDarkOrange1;

        public const HexColor colorDarkOrange2 = HexColor.colorDarkOrange2;

        public const HexColor colorDarkOrange3 = HexColor.colorDarkOrange3;

        public const HexColor colorDarkOrange4 = HexColor.colorDarkOrange4;

        public const HexColor colorDarkOrchid = HexColor.colorDarkOrchid;

        public const HexColor colorDarkOrchid1 = HexColor.colorDarkOrchid1;

        public const HexColor colorDarkOrchid2 = HexColor.colorDarkOrchid2;

        public const HexColor colorDarkOrchid3 = HexColor.colorDarkOrchid3;

        public const HexColor colorDarkOrchid4 = HexColor.colorDarkOrchid4;

        public const HexColor colorDarkRed = HexColor.colorDarkRed;

        public const HexColor colorDarkSalmon = HexColor.colorDarkSalmon;

        public const HexColor colorDarkSeaGreen = HexColor.colorDarkSeaGreen;

        public const HexColor colorDarkSeaGreen1 = HexColor.colorDarkSeaGreen1;

        public const HexColor colorDarkSeaGreen2 = HexColor.colorDarkSeaGreen2;

        public const HexColor colorDarkSeaGreen3 = HexColor.colorDarkSeaGreen3;

        public const HexColor colorDarkSeaGreen4 = HexColor.colorDarkSeaGreen4;

        public const HexColor colorDarkSlateBlue = HexColor.colorDarkSlateBlue;

        public const HexColor colorDarkSlateGray = HexColor.colorDarkSlateGray;

        public const HexColor colorDarkSlateGray1 = HexColor.colorDarkSlateGray1;

        public const HexColor colorDarkSlateGray2 = HexColor.colorDarkSlateGray2;

        public const HexColor colorDarkSlateGray3 = HexColor.colorDarkSlateGray3;

        public const HexColor colorDarkSlateGray4 = HexColor.colorDarkSlateGray4;

        public const HexColor colorDarkTurquoise = HexColor.colorDarkTurquoise;

        public const HexColor colorDarkViolet = HexColor.colorDarkViolet;

        public const HexColor colorDeepPink = HexColor.colorDeepPink;

        public const HexColor colorDeepPink1 = HexColor.colorDeepPink1;

        public const HexColor colorDeepPink2 = HexColor.colorDeepPink2;

        public const HexColor colorDeepPink3 = HexColor.colorDeepPink3;

        public const HexColor colorDeepPink4 = HexColor.colorDeepPink4;

        public const HexColor colorDeepSkyBlue = HexColor.colorDeepSkyBlue;

        public const HexColor colorDeepSkyBlue1 = HexColor.colorDeepSkyBlue1;

        public const HexColor colorDeepSkyBlue2 = HexColor.colorDeepSkyBlue2;

        public const HexColor colorDeepSkyBlue3 = HexColor.colorDeepSkyBlue3;

        public const HexColor colorDeepSkyBlue4 = HexColor.colorDeepSkyBlue4;

        public const HexColor colorDimGray = HexColor.colorDimGray;

        public const HexColor colorDodgerBlue = HexColor.colorDodgerBlue;

        public const HexColor colorDodgerBlue1 = HexColor.colorDodgerBlue1;

        public const HexColor colorDodgerBlue2 = HexColor.colorDodgerBlue2;

        public const HexColor colorDodgerBlue3 = HexColor.colorDodgerBlue3;

        public const HexColor colorDodgerBlue4 = HexColor.colorDodgerBlue4;

        public const HexColor colorFirebrick = HexColor.colorFirebrick;

        public const HexColor colorFirebrick1 = HexColor.colorFirebrick1;

        public const HexColor colorFirebrick2 = HexColor.colorFirebrick2;

        public const HexColor colorFirebrick3 = HexColor.colorFirebrick3;

        public const HexColor colorFirebrick4 = HexColor.colorFirebrick4;

        public const HexColor colorFloralWhite = HexColor.colorFloralWhite;

        public const HexColor colorForestGreen = HexColor.colorForestGreen;

        public const HexColor colorFuchsia = HexColor.colorFuchsia;

        public const HexColor colorGainsboro = HexColor.colorGainsboro;

        public const HexColor colorGhostWhite = HexColor.colorGhostWhite;

        public const HexColor colorGold = HexColor.colorGold;

        public const HexColor colorGold1 = HexColor.colorGold1;

        public const HexColor colorGold2 = HexColor.colorGold2;

        public const HexColor colorGold3 = HexColor.colorGold3;

        public const HexColor colorGold4 = HexColor.colorGold4;

        public const HexColor colorGoldenrod = HexColor.colorGoldenrod;

        public const HexColor colorGoldenrod1 = HexColor.colorGoldenrod1;

        public const HexColor colorGoldenrod2 = HexColor.colorGoldenrod2;

        public const HexColor colorGoldenrod3 = HexColor.colorGoldenrod3;

        public const HexColor colorGoldenrod4 = HexColor.colorGoldenrod4;

        public const HexColor colorGray = HexColor.colorGray;

        public const HexColor colorGray0 = HexColor.colorGray0;

        public const HexColor colorGray1 = HexColor.colorGray1;

        public const HexColor colorGray10 = HexColor.colorGray10;

        public const HexColor colorGray100 = HexColor.colorGray100;

        public const HexColor colorGray11 = HexColor.colorGray11;

        public const HexColor colorGray12 = HexColor.colorGray12;

        public const HexColor colorGray13 = HexColor.colorGray13;

        public const HexColor colorGray14 = HexColor.colorGray14;

        public const HexColor colorGray15 = HexColor.colorGray15;

        public const HexColor colorGray16 = HexColor.colorGray16;

        public const HexColor colorGray17 = HexColor.colorGray17;

        public const HexColor colorGray18 = HexColor.colorGray18;

        public const HexColor colorGray19 = HexColor.colorGray19;

        public const HexColor colorGray2 = HexColor.colorGray2;

        public const HexColor colorGray20 = HexColor.colorGray20;

        public const HexColor colorGray21 = HexColor.colorGray21;

        public const HexColor colorGray22 = HexColor.colorGray22;

        public const HexColor colorGray23 = HexColor.colorGray23;

        public const HexColor colorGray24 = HexColor.colorGray24;

        public const HexColor colorGray25 = HexColor.colorGray25;

        public const HexColor colorGray26 = HexColor.colorGray26;

        public const HexColor colorGray27 = HexColor.colorGray27;

        public const HexColor colorGray28 = HexColor.colorGray28;

        public const HexColor colorGray29 = HexColor.colorGray29;

        public const HexColor colorGray3 = HexColor.colorGray3;

        public const HexColor colorGray30 = HexColor.colorGray30;

        public const HexColor colorGray31 = HexColor.colorGray31;

        public const HexColor colorGray32 = HexColor.colorGray32;

        public const HexColor colorGray33 = HexColor.colorGray33;

        public const HexColor colorGray34 = HexColor.colorGray34;

        public const HexColor colorGray35 = HexColor.colorGray35;

        public const HexColor colorGray36 = HexColor.colorGray36;

        public const HexColor colorGray37 = HexColor.colorGray37;

        public const HexColor colorGray38 = HexColor.colorGray38;

        public const HexColor colorGray39 = HexColor.colorGray39;

        public const HexColor colorGray4 = HexColor.colorGray4;

        public const HexColor colorGray40 = HexColor.colorGray40;

        public const HexColor colorGray41 = HexColor.colorGray41;

        public const HexColor colorGray42 = HexColor.colorGray42;

        public const HexColor colorGray43 = HexColor.colorGray43;

        public const HexColor colorGray44 = HexColor.colorGray44;

        public const HexColor colorGray45 = HexColor.colorGray45;

        public const HexColor colorGray46 = HexColor.colorGray46;

        public const HexColor colorGray47 = HexColor.colorGray47;

        public const HexColor colorGray48 = HexColor.colorGray48;

        public const HexColor colorGray49 = HexColor.colorGray49;

        public const HexColor colorGray5 = HexColor.colorGray5;

        public const HexColor colorGray50 = HexColor.colorGray50;

        public const HexColor colorGray51 = HexColor.colorGray51;

        public const HexColor colorGray52 = HexColor.colorGray52;

        public const HexColor colorGray53 = HexColor.colorGray53;

        public const HexColor colorGray54 = HexColor.colorGray54;

        public const HexColor colorGray55 = HexColor.colorGray55;

        public const HexColor colorGray56 = HexColor.colorGray56;

        public const HexColor colorGray57 = HexColor.colorGray57;

        public const HexColor colorGray58 = HexColor.colorGray58;

        public const HexColor colorGray59 = HexColor.colorGray59;

        public const HexColor colorGray6 = HexColor.colorGray6;

        public const HexColor colorGray60 = HexColor.colorGray60;

        public const HexColor colorGray61 = HexColor.colorGray61;

        public const HexColor colorGray62 = HexColor.colorGray62;

        public const HexColor colorGray63 = HexColor.colorGray63;

        public const HexColor colorGray64 = HexColor.colorGray64;

        public const HexColor colorGray65 = HexColor.colorGray65;

        public const HexColor colorGray66 = HexColor.colorGray66;

        public const HexColor colorGray67 = HexColor.colorGray67;

        public const HexColor colorGray68 = HexColor.colorGray68;

        public const HexColor colorGray69 = HexColor.colorGray69;

        public const HexColor colorGray7 = HexColor.colorGray7;

        public const HexColor colorGray70 = HexColor.colorGray70;

        public const HexColor colorGray71 = HexColor.colorGray71;

        public const HexColor colorGray72 = HexColor.colorGray72;

        public const HexColor colorGray73 = HexColor.colorGray73;

        public const HexColor colorGray74 = HexColor.colorGray74;

        public const HexColor colorGray75 = HexColor.colorGray75;

        public const HexColor colorGray76 = HexColor.colorGray76;

        public const HexColor colorGray77 = HexColor.colorGray77;

        public const HexColor colorGray78 = HexColor.colorGray78;

        public const HexColor colorGray79 = HexColor.colorGray79;

        public const HexColor colorGray8 = HexColor.colorGray8;

        public const HexColor colorGray80 = HexColor.colorGray80;

        public const HexColor colorGray81 = HexColor.colorGray81;

        public const HexColor colorGray82 = HexColor.colorGray82;

        public const HexColor colorGray83 = HexColor.colorGray83;

        public const HexColor colorGray84 = HexColor.colorGray84;

        public const HexColor colorGray85 = HexColor.colorGray85;

        public const HexColor colorGray86 = HexColor.colorGray86;

        public const HexColor colorGray87 = HexColor.colorGray87;

        public const HexColor colorGray88 = HexColor.colorGray88;

        public const HexColor colorGray89 = HexColor.colorGray89;

        public const HexColor colorGray9 = HexColor.colorGray9;

        public const HexColor colorGray90 = HexColor.colorGray90;

        public const HexColor colorGray91 = HexColor.colorGray91;

        public const HexColor colorGray92 = HexColor.colorGray92;

        public const HexColor colorGray93 = HexColor.colorGray93;

        public const HexColor colorGray94 = HexColor.colorGray94;

        public const HexColor colorGray95 = HexColor.colorGray95;

        public const HexColor colorGray96 = HexColor.colorGray96;

        public const HexColor colorGray97 = HexColor.colorGray97;

        public const HexColor colorGray98 = HexColor.colorGray98;

        public const HexColor colorGray99 = HexColor.colorGray99;

        public const HexColor colorGreen = HexColor.colorGreen;

        public const HexColor colorGreen1 = HexColor.colorGreen1;

        public const HexColor colorGreen2 = HexColor.colorGreen2;

        public const HexColor colorGreen3 = HexColor.colorGreen3;

        public const HexColor colorGreen4 = HexColor.colorGreen4;

        public const HexColor colorGreenYellow = HexColor.colorGreenYellow;

        public const HexColor colorHoneydew = HexColor.colorHoneydew;

        public const HexColor colorHoneydew1 = HexColor.colorHoneydew1;

        public const HexColor colorHoneydew2 = HexColor.colorHoneydew2;

        public const HexColor colorHoneydew3 = HexColor.colorHoneydew3;

        public const HexColor colorHoneydew4 = HexColor.colorHoneydew4;

        public const HexColor colorHotPink = HexColor.colorHotPink;

        public const HexColor colorHotPink1 = HexColor.colorHotPink1;

        public const HexColor colorHotPink2 = HexColor.colorHotPink2;

        public const HexColor colorHotPink3 = HexColor.colorHotPink3;

        public const HexColor colorHotPink4 = HexColor.colorHotPink4;

        public const HexColor colorIndianRed = HexColor.colorIndianRed;

        public const HexColor colorIndianRed1 = HexColor.colorIndianRed1;

        public const HexColor colorIndianRed2 = HexColor.colorIndianRed2;

        public const HexColor colorIndianRed3 = HexColor.colorIndianRed3;

        public const HexColor colorIndianRed4 = HexColor.colorIndianRed4;

        public const HexColor colorIndigo = HexColor.colorIndigo;

        public const HexColor colorIvory = HexColor.colorIvory;

        public const HexColor colorIvory1 = HexColor.colorIvory1;

        public const HexColor colorIvory2 = HexColor.colorIvory2;

        public const HexColor colorIvory3 = HexColor.colorIvory3;

        public const HexColor colorIvory4 = HexColor.colorIvory4;

        public const HexColor colorKhaki = HexColor.colorKhaki;

        public const HexColor colorKhaki1 = HexColor.colorKhaki1;

        public const HexColor colorKhaki2 = HexColor.colorKhaki2;

        public const HexColor colorKhaki3 = HexColor.colorKhaki3;

        public const HexColor colorKhaki4 = HexColor.colorKhaki4;

        public const HexColor colorLavender = HexColor.colorLavender;

        public const HexColor colorLavenderBlush = HexColor.colorLavenderBlush;

        public const HexColor colorLavenderBlush1 = HexColor.colorLavenderBlush1;

        public const HexColor colorLavenderBlush2 = HexColor.colorLavenderBlush2;

        public const HexColor colorLavenderBlush3 = HexColor.colorLavenderBlush3;

        public const HexColor colorLavenderBlush4 = HexColor.colorLavenderBlush4;

        public const HexColor colorLawnGreen = HexColor.colorLawnGreen;

        public const HexColor colorLemonChiffon = HexColor.colorLemonChiffon;

        public const HexColor colorLemonChiffon1 = HexColor.colorLemonChiffon1;

        public const HexColor colorLemonChiffon2 = HexColor.colorLemonChiffon2;

        public const HexColor colorLemonChiffon3 = HexColor.colorLemonChiffon3;

        public const HexColor colorLemonChiffon4 = HexColor.colorLemonChiffon4;

        public const HexColor colorLightBlue = HexColor.colorLightBlue;

        public const HexColor colorLightBlue1 = HexColor.colorLightBlue1;

        public const HexColor colorLightBlue2 = HexColor.colorLightBlue2;

        public const HexColor colorLightBlue3 = HexColor.colorLightBlue3;

        public const HexColor colorLightBlue4 = HexColor.colorLightBlue4;

        public const HexColor colorLightCoral = HexColor.colorLightCoral;

        public const HexColor colorLightCyan = HexColor.colorLightCyan;

        public const HexColor colorLightCyan1 = HexColor.colorLightCyan1;

        public const HexColor colorLightCyan2 = HexColor.colorLightCyan2;

        public const HexColor colorLightCyan3 = HexColor.colorLightCyan3;

        public const HexColor colorLightCyan4 = HexColor.colorLightCyan4;

        public const HexColor colorLightGoldenrod = HexColor.colorLightGoldenrod;

        public const HexColor colorLightGoldenrod1 = HexColor.colorLightGoldenrod1;

        public const HexColor colorLightGoldenrod2 = HexColor.colorLightGoldenrod2;

        public const HexColor colorLightGoldenrod3 = HexColor.colorLightGoldenrod3;

        public const HexColor colorLightGoldenrod4 = HexColor.colorLightGoldenrod4;

        public const HexColor colorLightGoldenrodYellow = HexColor.colorLightGoldenrodYellow;

        public const HexColor colorLightGray = HexColor.colorLightGray;

        public const HexColor colorLightGreen = HexColor.colorLightGreen;

        public const HexColor colorLightPink = HexColor.colorLightPink;

        public const HexColor colorLightPink1 = HexColor.colorLightPink1;

        public const HexColor colorLightPink2 = HexColor.colorLightPink2;

        public const HexColor colorLightPink3 = HexColor.colorLightPink3;

        public const HexColor colorLightPink4 = HexColor.colorLightPink4;

        public const HexColor colorLightSalmon = HexColor.colorLightSalmon;

        public const HexColor colorLightSalmon1 = HexColor.colorLightSalmon1;

        public const HexColor colorLightSalmon2 = HexColor.colorLightSalmon2;

        public const HexColor colorLightSalmon3 = HexColor.colorLightSalmon3;

        public const HexColor colorLightSalmon4 = HexColor.colorLightSalmon4;

        public const HexColor colorLightSeaGreen = HexColor.colorLightSeaGreen;

        public const HexColor colorLightSkyBlue = HexColor.colorLightSkyBlue;

        public const HexColor colorLightSkyBlue1 = HexColor.colorLightSkyBlue1;

        public const HexColor colorLightSkyBlue2 = HexColor.colorLightSkyBlue2;

        public const HexColor colorLightSkyBlue3 = HexColor.colorLightSkyBlue3;

        public const HexColor colorLightSkyBlue4 = HexColor.colorLightSkyBlue4;

        public const HexColor colorLightSlateBlue = HexColor.colorLightSlateBlue;

        public const HexColor colorLightSlateGray = HexColor.colorLightSlateGray;

        public const HexColor colorLightSteelBlue = HexColor.colorLightSteelBlue;

        public const HexColor colorLightSteelBlue1 = HexColor.colorLightSteelBlue1;

        public const HexColor colorLightSteelBlue2 = HexColor.colorLightSteelBlue2;

        public const HexColor colorLightSteelBlue3 = HexColor.colorLightSteelBlue3;

        public const HexColor colorLightSteelBlue4 = HexColor.colorLightSteelBlue4;

        public const HexColor colorLightYellow = HexColor.colorLightYellow;

        public const HexColor colorLightYellow1 = HexColor.colorLightYellow1;

        public const HexColor colorLightYellow2 = HexColor.colorLightYellow2;

        public const HexColor colorLightYellow3 = HexColor.colorLightYellow3;

        public const HexColor colorLightYellow4 = HexColor.colorLightYellow4;

        public const HexColor colorLime = HexColor.colorLime;

        public const HexColor colorLimeGreen = HexColor.colorLimeGreen;

        public const HexColor colorLinen = HexColor.colorLinen;

        public const HexColor colorMagenta = HexColor.colorMagenta;

        public const HexColor colorMagenta1 = HexColor.colorMagenta1;

        public const HexColor colorMagenta2 = HexColor.colorMagenta2;

        public const HexColor colorMagenta3 = HexColor.colorMagenta3;

        public const HexColor colorMagenta4 = HexColor.colorMagenta4;

        public const HexColor colorMaroon = HexColor.colorMaroon;

        public const HexColor colorMaroon1 = HexColor.colorMaroon1;

        public const HexColor colorMaroon2 = HexColor.colorMaroon2;

        public const HexColor colorMaroon3 = HexColor.colorMaroon3;

        public const HexColor colorMaroon4 = HexColor.colorMaroon4;

        public const HexColor colorMediumAquamarine = HexColor.colorMediumAquamarine;

        public const HexColor colorMediumBlue = HexColor.colorMediumBlue;

        public const HexColor colorMediumOrchid = HexColor.colorMediumOrchid;

        public const HexColor colorMediumOrchid1 = HexColor.colorMediumOrchid1;

        public const HexColor colorMediumOrchid2 = HexColor.colorMediumOrchid2;

        public const HexColor colorMediumOrchid3 = HexColor.colorMediumOrchid3;

        public const HexColor colorMediumOrchid4 = HexColor.colorMediumOrchid4;

        public const HexColor colorMediumPurple = HexColor.colorMediumPurple;

        public const HexColor colorMediumPurple1 = HexColor.colorMediumPurple1;

        public const HexColor colorMediumPurple2 = HexColor.colorMediumPurple2;

        public const HexColor colorMediumPurple3 = HexColor.colorMediumPurple3;

        public const HexColor colorMediumPurple4 = HexColor.colorMediumPurple4;

        public const HexColor colorMediumSeaGreen = HexColor.colorMediumSeaGreen;

        public const HexColor colorMediumSlateBlue = HexColor.colorMediumSlateBlue;

        public const HexColor colorMediumSpringGreen = HexColor.colorMediumSpringGreen;

        public const HexColor colorMediumTurquoise = HexColor.colorMediumTurquoise;

        public const HexColor colorMediumVioletRed = HexColor.colorMediumVioletRed;

        public const HexColor colorMidnightBlue = HexColor.colorMidnightBlue;

        public const HexColor colorMintCream = HexColor.colorMintCream;

        public const HexColor colorMistyRose = HexColor.colorMistyRose;

        public const HexColor colorMistyRose1 = HexColor.colorMistyRose1;

        public const HexColor colorMistyRose2 = HexColor.colorMistyRose2;

        public const HexColor colorMistyRose3 = HexColor.colorMistyRose3;

        public const HexColor colorMistyRose4 = HexColor.colorMistyRose4;

        public const HexColor colorMoccasin = HexColor.colorMoccasin;

        public const HexColor colorNavajoWhite = HexColor.colorNavajoWhite;

        public const HexColor colorNavajoWhite1 = HexColor.colorNavajoWhite1;

        public const HexColor colorNavajoWhite2 = HexColor.colorNavajoWhite2;

        public const HexColor colorNavajoWhite3 = HexColor.colorNavajoWhite3;

        public const HexColor colorNavajoWhite4 = HexColor.colorNavajoWhite4;

        public const HexColor colorNavy = HexColor.colorNavy;

        public const HexColor colorNavyBlue = HexColor.colorNavyBlue;

        public const HexColor colorOldLace = HexColor.colorOldLace;

        public const HexColor colorOlive = HexColor.colorOlive;

        public const HexColor colorOliveDrab = HexColor.colorOliveDrab;

        public const HexColor colorOliveDrab1 = HexColor.colorOliveDrab1;

        public const HexColor colorOliveDrab2 = HexColor.colorOliveDrab2;

        public const HexColor colorOliveDrab3 = HexColor.colorOliveDrab3;

        public const HexColor colorOliveDrab4 = HexColor.colorOliveDrab4;

        public const HexColor colorOrange = HexColor.colorOrange;

        public const HexColor colorOrange1 = HexColor.colorOrange1;

        public const HexColor colorOrange2 = HexColor.colorOrange2;

        public const HexColor colorOrange3 = HexColor.colorOrange3;

        public const HexColor colorOrange4 = HexColor.colorOrange4;

        public const HexColor colorOrangeRed = HexColor.colorOrangeRed;

        public const HexColor colorOrangeRed1 = HexColor.colorOrangeRed1;

        public const HexColor colorOrangeRed2 = HexColor.colorOrangeRed2;

        public const HexColor colorOrangeRed3 = HexColor.colorOrangeRed3;

        public const HexColor colorOrangeRed4 = HexColor.colorOrangeRed4;

        public const HexColor colorOrchid = HexColor.colorOrchid;

        public const HexColor colorOrchid1 = HexColor.colorOrchid1;

        public const HexColor colorOrchid2 = HexColor.colorOrchid2;

        public const HexColor colorOrchid3 = HexColor.colorOrchid3;

        public const HexColor colorOrchid4 = HexColor.colorOrchid4;

        public const HexColor colorPaleGoldenrod = HexColor.colorPaleGoldenrod;

        public const HexColor colorPaleGreen = HexColor.colorPaleGreen;

        public const HexColor colorPaleGreen1 = HexColor.colorPaleGreen1;

        public const HexColor colorPaleGreen2 = HexColor.colorPaleGreen2;

        public const HexColor colorPaleGreen3 = HexColor.colorPaleGreen3;

        public const HexColor colorPaleGreen4 = HexColor.colorPaleGreen4;

        public const HexColor colorPaleTurquoise = HexColor.colorPaleTurquoise;

        public const HexColor colorPaleTurquoise1 = HexColor.colorPaleTurquoise1;

        public const HexColor colorPaleTurquoise2 = HexColor.colorPaleTurquoise2;

        public const HexColor colorPaleTurquoise3 = HexColor.colorPaleTurquoise3;

        public const HexColor colorPaleTurquoise4 = HexColor.colorPaleTurquoise4;

        public const HexColor colorPaleVioletRed = HexColor.colorPaleVioletRed;

        public const HexColor colorPaleVioletRed1 = HexColor.colorPaleVioletRed1;

        public const HexColor colorPaleVioletRed2 = HexColor.colorPaleVioletRed2;

        public const HexColor colorPaleVioletRed3 = HexColor.colorPaleVioletRed3;

        public const HexColor colorPaleVioletRed4 = HexColor.colorPaleVioletRed4;

        public const HexColor colorPapayaWhip = HexColor.colorPapayaWhip;

        public const HexColor colorPeachPuff = HexColor.colorPeachPuff;

        public const HexColor colorPeachPuff1 = HexColor.colorPeachPuff1;

        public const HexColor colorPeachPuff2 = HexColor.colorPeachPuff2;

        public const HexColor colorPeachPuff3 = HexColor.colorPeachPuff3;

        public const HexColor colorPeachPuff4 = HexColor.colorPeachPuff4;

        public const HexColor colorPeru = HexColor.colorPeru;

        public const HexColor colorPink = HexColor.colorPink;

        public const HexColor colorPink1 = HexColor.colorPink1;

        public const HexColor colorPink2 = HexColor.colorPink2;

        public const HexColor colorPink3 = HexColor.colorPink3;

        public const HexColor colorPink4 = HexColor.colorPink4;

        public const HexColor colorPlum = HexColor.colorPlum;

        public const HexColor colorPlum1 = HexColor.colorPlum1;

        public const HexColor colorPlum2 = HexColor.colorPlum2;

        public const HexColor colorPlum3 = HexColor.colorPlum3;

        public const HexColor colorPlum4 = HexColor.colorPlum4;

        public const HexColor colorPowderBlue = HexColor.colorPowderBlue;

        public const HexColor colorPurple = HexColor.colorPurple;

        public const HexColor colorPurple1 = HexColor.colorPurple1;

        public const HexColor colorPurple2 = HexColor.colorPurple2;

        public const HexColor colorPurple3 = HexColor.colorPurple3;

        public const HexColor colorPurple4 = HexColor.colorPurple4;

        public const HexColor colorRebeccaPurple = HexColor.colorRebeccaPurple;

        public const HexColor colorRed = HexColor.colorRed;

        public const HexColor colorRed1 = HexColor.colorRed1;

        public const HexColor colorRed2 = HexColor.colorRed2;

        public const HexColor colorRed3 = HexColor.colorRed3;

        public const HexColor colorRed4 = HexColor.colorRed4;

        public const HexColor colorRosyBrown = HexColor.colorRosyBrown;

        public const HexColor colorRosyBrown1 = HexColor.colorRosyBrown1;

        public const HexColor colorRosyBrown2 = HexColor.colorRosyBrown2;

        public const HexColor colorRosyBrown3 = HexColor.colorRosyBrown3;

        public const HexColor colorRosyBrown4 = HexColor.colorRosyBrown4;

        public const HexColor colorRoyalBlue = HexColor.colorRoyalBlue;

        public const HexColor colorRoyalBlue1 = HexColor.colorRoyalBlue1;

        public const HexColor colorRoyalBlue2 = HexColor.colorRoyalBlue2;

        public const HexColor colorRoyalBlue3 = HexColor.colorRoyalBlue3;

        public const HexColor colorRoyalBlue4 = HexColor.colorRoyalBlue4;

        public const HexColor colorSaddleBrown = HexColor.colorSaddleBrown;

        public const HexColor colorSalmon = HexColor.colorSalmon;

        public const HexColor colorSalmon1 = HexColor.colorSalmon1;

        public const HexColor colorSalmon2 = HexColor.colorSalmon2;

        public const HexColor colorSalmon3 = HexColor.colorSalmon3;

        public const HexColor colorSalmon4 = HexColor.colorSalmon4;

        public const HexColor colorSandyBrown = HexColor.colorSandyBrown;

        public const HexColor colorSeaGreen = HexColor.colorSeaGreen;

        public const HexColor colorSeaGreen1 = HexColor.colorSeaGreen1;

        public const HexColor colorSeaGreen2 = HexColor.colorSeaGreen2;

        public const HexColor colorSeaGreen3 = HexColor.colorSeaGreen3;

        public const HexColor colorSeaGreen4 = HexColor.colorSeaGreen4;

        public const HexColor colorSeashell = HexColor.colorSeashell;

        public const HexColor colorSeashell1 = HexColor.colorSeashell1;

        public const HexColor colorSeashell2 = HexColor.colorSeashell2;

        public const HexColor colorSeashell3 = HexColor.colorSeashell3;

        public const HexColor colorSeashell4 = HexColor.colorSeashell4;

        public const HexColor colorSienna = HexColor.colorSienna;

        public const HexColor colorSienna1 = HexColor.colorSienna1;

        public const HexColor colorSienna2 = HexColor.colorSienna2;

        public const HexColor colorSienna3 = HexColor.colorSienna3;

        public const HexColor colorSienna4 = HexColor.colorSienna4;

        public const HexColor colorSilver = HexColor.colorSilver;

        public const HexColor colorSkyBlue = HexColor.colorSkyBlue;

        public const HexColor colorSkyBlue1 = HexColor.colorSkyBlue1;

        public const HexColor colorSkyBlue2 = HexColor.colorSkyBlue2;

        public const HexColor colorSkyBlue3 = HexColor.colorSkyBlue3;

        public const HexColor colorSkyBlue4 = HexColor.colorSkyBlue4;

        public const HexColor colorSlateBlue = HexColor.colorSlateBlue;

        public const HexColor colorSlateBlue1 = HexColor.colorSlateBlue1;

        public const HexColor colorSlateBlue2 = HexColor.colorSlateBlue2;

        public const HexColor colorSlateBlue3 = HexColor.colorSlateBlue3;

        public const HexColor colorSlateBlue4 = HexColor.colorSlateBlue4;

        public const HexColor colorSlateGray = HexColor.colorSlateGray;

        public const HexColor colorSlateGray1 = HexColor.colorSlateGray1;

        public const HexColor colorSlateGray2 = HexColor.colorSlateGray2;

        public const HexColor colorSlateGray3 = HexColor.colorSlateGray3;

        public const HexColor colorSlateGray4 = HexColor.colorSlateGray4;

        public const HexColor colorSnow = HexColor.colorSnow;

        public const HexColor colorSnow1 = HexColor.colorSnow1;

        public const HexColor colorSnow2 = HexColor.colorSnow2;

        public const HexColor colorSnow3 = HexColor.colorSnow3;

        public const HexColor colorSnow4 = HexColor.colorSnow4;

        public const HexColor colorSpringGreen = HexColor.colorSpringGreen;

        public const HexColor colorSpringGreen1 = HexColor.colorSpringGreen1;

        public const HexColor colorSpringGreen2 = HexColor.colorSpringGreen2;

        public const HexColor colorSpringGreen3 = HexColor.colorSpringGreen3;

        public const HexColor colorSpringGreen4 = HexColor.colorSpringGreen4;

        public const HexColor colorSteelBlue = HexColor.colorSteelBlue;

        public const HexColor colorSteelBlue1 = HexColor.colorSteelBlue1;

        public const HexColor colorSteelBlue2 = HexColor.colorSteelBlue2;

        public const HexColor colorSteelBlue3 = HexColor.colorSteelBlue3;

        public const HexColor colorSteelBlue4 = HexColor.colorSteelBlue4;

        public const HexColor colorTan = HexColor.colorTan;

        public const HexColor colorTan1 = HexColor.colorTan1;

        public const HexColor colorTan2 = HexColor.colorTan2;

        public const HexColor colorTan3 = HexColor.colorTan3;

        public const HexColor colorTan4 = HexColor.colorTan4;

        public const HexColor colorTeal = HexColor.colorTeal;

        public const HexColor colorThistle = HexColor.colorThistle;

        public const HexColor colorThistle1 = HexColor.colorThistle1;

        public const HexColor colorThistle2 = HexColor.colorThistle2;

        public const HexColor colorThistle3 = HexColor.colorThistle3;

        public const HexColor colorThistle4 = HexColor.colorThistle4;

        public const HexColor colorTomato = HexColor.colorTomato;

        public const HexColor colorTomato1 = HexColor.colorTomato1;

        public const HexColor colorTomato2 = HexColor.colorTomato2;

        public const HexColor colorTomato3 = HexColor.colorTomato3;

        public const HexColor colorTomato4 = HexColor.colorTomato4;

        public const HexColor colorTurquoise = HexColor.colorTurquoise;

        public const HexColor colorTurquoise1 = HexColor.colorTurquoise1;

        public const HexColor colorTurquoise2 = HexColor.colorTurquoise2;

        public const HexColor colorTurquoise3 = HexColor.colorTurquoise3;

        public const HexColor colorTurquoise4 = HexColor.colorTurquoise4;

        public const HexColor colorViolet = HexColor.colorViolet;

        public const HexColor colorVioletRed = HexColor.colorVioletRed;

        public const HexColor colorVioletRed1 = HexColor.colorVioletRed1;

        public const HexColor colorVioletRed2 = HexColor.colorVioletRed2;

        public const HexColor colorVioletRed3 = HexColor.colorVioletRed3;

        public const HexColor colorVioletRed4 = HexColor.colorVioletRed4;

        public const HexColor colorWebGray = HexColor.colorWebGray;

        public const HexColor colorWebGreen = HexColor.colorWebGreen;

        public const HexColor colorWebMaroon = HexColor.colorWebMaroon;

        public const HexColor colorWebPurple = HexColor.colorWebPurple;

        public const HexColor colorWheat = HexColor.colorWheat;

        public const HexColor colorWheat1 = HexColor.colorWheat1;

        public const HexColor colorWheat2 = HexColor.colorWheat2;

        public const HexColor colorWheat3 = HexColor.colorWheat3;

        public const HexColor colorWheat4 = HexColor.colorWheat4;

        public const HexColor colorWhite = HexColor.colorWhite;

        public const HexColor colorWhiteSmoke = HexColor.colorWhiteSmoke;

        public const HexColor colorX11Gray = HexColor.colorX11Gray;

        public const HexColor colorX11Green = HexColor.colorX11Green;

        public const HexColor colorX11Maroon = HexColor.colorX11Maroon;

        public const HexColor colorX11Purple = HexColor.colorX11Purple;

        public const HexColor colorYellow = HexColor.colorYellow;

        public const HexColor colorYellow1 = HexColor.colorYellow1;

        public const HexColor colorYellow2 = HexColor.colorYellow2;

        public const HexColor colorYellow3 = HexColor.colorYellow3;

        public const HexColor colorYellow4 = HexColor.colorYellow4;

        public const HexColor colorYellowGreen = HexColor.colorYellowGreen;

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

        public const float aabbMargin = 0.1f;

        public const float angularSleepTolerance = 0.03490659f;

        public const float angularSlop = 0.03490659f;

        public const int defaultCategoryBits = 1;

        public const uint defaultMaskBits = 4294967295;

        public const int graphColorCount = 12;

        public const float huge = 100000f;

        public const float lengthUnitsPerMeter = 1f;

        public const float linearSleepTolerance = 0.01f;

        public const float linearSlop = 0.005f;

        public const int maxPolygonVertices = 8;

        public const float maxRotation = 0.7853982f;

        public const float maxTranslation = 4f;

        public const int maxWorkers = 64;

        public const int maxWorlds = 128;

        public const float pi = 3.1415927f;

        public const float speculativeDistance = 0.02f;

        public const float timeToSleep = 0.5f;

        public partial class BindgenInternal
        {
            public const string DllImportPath = "box2c";

            static BindgenInternal()
            {
                DllFilePaths = new System.Collections.Generic.List<string>
                {
                    "box2c",
                    "runtimes/linux-x64/native/box2c",
                    "runtimes/linux-arm64/native/box2c",
                    "runtimes/osx-x64/native/box2c",
                    "runtimes/osx-arm64/native/box2c",
                    "runtimes/win-x64/native/box2c",
                    "runtimes/win-arm64/native/box2c"
                };
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Usage", "SYSLIB1054")]
        public partial class BindgenInternal
        {
            public static readonly System.Collections.Generic.List<string> DllFilePaths;

            public static System.IntPtr _libraryHandle = System.IntPtr.Zero;

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
#if NET5_0_OR_GREATER
            return System.Runtime.InteropServices.NativeLibrary.TryLoad(path, out handle);
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
#if NET5_0_OR_GREATER
            return System.Runtime.InteropServices.NativeLibrary.GetExport(_libraryHandle, symbol);
#else
                if (IsLinux)
                {
                    GetLastErrorLinux();
                    System.IntPtr handle = GetExportLinux(_libraryHandle, symbol);
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
                    System.IntPtr handle = GetExportOsx(_libraryHandle, symbol);
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
                    System.IntPtr handle = GetExportWindows(_libraryHandle, symbol);
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
                    string searchDir = System.IO.Path.IsPathRooted(dllFilePath) ? System.IO.Path.GetFullPath(parentDir) + "/" : System.IO.Path.GetFullPath(System.AppDomain.CurrentDomain.BaseDirectory + parentDir) + "/";
                    if (TryLoad($"{searchDir}{fileName}", out _libraryHandle))
                        return;
                    if (TryLoad($"{searchDir}{fileName}{fileExtension}", out _libraryHandle))
                        return;
                    if (TryLoad($"{searchDir}lib{fileName}", out _libraryHandle))
                        return;
                    if (TryLoad($"{searchDir}lib{fileName}{fileExtension}", out _libraryHandle))
                        return;
                    if (!fileName.StartsWith("lib") || fileName == "lib")
                        continue;
                    string unprefixed = fileName.Substring(4);
                    if (TryLoad($"{searchDir}{unprefixed}", out _libraryHandle))
                        return;
                    if (TryLoad($"{searchDir}{unprefixed}{fileExtension}", out _libraryHandle))
                        return;
                }

#if NET7_0_OR_GREATER
                _libraryHandle = System.Runtime.InteropServices.NativeLibrary.GetMainProgramHandle();
#else
                if (IsLinux)
                    _libraryHandle = LoadLibraryLinux(null, 0x101);
                else if (IsOsx)
                    _libraryHandle = LoadLibraryOsx(null, 0x101);
                else if (IsWindows)
                    _libraryHandle = GetModuleHandle(null);
#endif
            }

            public static void* LoadDllSymbol(string variableSymbol, out void* field)
            {
                if (_libraryHandle == System.IntPtr.Zero)
                    ResolveLibrary();
                return field = (void*)GetExport(variableSymbol);
            }
        }
    }
}
#pragma warning restore CS9084
#nullable disable
