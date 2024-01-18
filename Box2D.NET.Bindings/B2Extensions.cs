#nullable enable
namespace Box2D.NET.Bindings
{
    public static partial class B2
    {
        public static readonly WorldDef DefaultWorldDef = new WorldDef
        {
            gravity = new Vec2 { x = 0.0f, y = -10.0f },
            restitutionThreshold = 1.0f * lengthUnitsPerMeter,
            contactPushoutVelocity = 3.0f * lengthUnitsPerMeter,
            contactHertz = 30.0f,
            contactDampingRatio = 1.0f,
            enableSleep = 1,
            bodyCapacity = 0,
            shapeCapacity = 0,
            contactCapacity = 0,
            jointCapacity = 0,
            arenaAllocatorCapacity = 1024 * 1024,
            workerCount = 0,
            enqueueTask = null,
            finishTask = null,
            userTaskContext = null
        };

        public static readonly BodyDef DefaultBodyDef = new BodyDef
        {
            type = staticBody,
            position = default,
            angle = 0.0f,
            linearVelocity = default,
            angularVelocity = 0.0f,
            linearDamping = 0.0f,
            angularDamping = 0.0f,
            gravityScale = 1.0f,
            userData = null,
            enableSleep = 1,
            isAwake = 1,
            fixedRotation = 0,
            isEnabled = 1
        };

        public static readonly Filter DefaultFilter = new Filter
        {
            categoryBits = 0x00000001,
            maskBits = 0xFFFFFFFF,
            groupIndex = 0
        };

        public static readonly QueryFilter DefaultQueryFilter = new QueryFilter
        {
            categoryBits = 0x00000001,
            maskBits = 0xFFFFFFFF
        };

        public static readonly ShapeDef DefaultShapeDef = new ShapeDef
        {
            userData = null,
            friction = 0.6f,
            restitution = 0.0f,
            density = 1.0f,
            filter = DefaultFilter,
            isSensor = 0,
            enableSensorEvents = 1,
            enableContactEvents = 1,
            enablePreSolveEvents = 0
        };

        public static readonly ChainDef DefaultChainDef = new ChainDef
        {
            points = null,
            count = 0,
            loop = 0,
            userData = null,
            friction = 0.6f,
            restitution = 0.0f,
            filter = DefaultFilter
        };
    }
}
