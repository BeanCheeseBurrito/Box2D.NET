using Box2D.NET.Bindings;

public static unsafe class HelloWorld
{
    public static void Main()
    {
        Console.WriteLine(B2.Normalize(default));
        // Create a world
        B2.WorldDef worldDef = B2.DefaultWorldDef();
        worldDef.gravity = new B2.Vec2 { x = 0, y = -9.81f };

        B2.WorldId worldId = B2.CreateWorld(&worldDef);

        // Create a body
        B2.BodyDef bodyDef = B2.DefaultBodyDef();
        bodyDef.type = B2.dynamicBody;

        B2.BodyId bodyId = B2.CreateBody(worldId, &bodyDef);

        // Create a shape
        B2.Polygon box = B2.MakeBox(1, 1);
        B2.ShapeDef shapeDef = B2.DefaultShapeDef();
        shapeDef.friction = 0.6f;
        shapeDef.density = 1.0f;

        B2.ShapeId shapeId = B2.CreatePolygonShape(bodyId, &shapeDef, &box);

        // Run the simulation
        const float timeStep = 1.0f / 60.0f;
        const int subStepCount = 4;

        for (int i = 0; i < 60; i++)
        {
            B2.WorldStep(worldId, timeStep, subStepCount);
            B2.Vec2 position = B2.BodyGetPosition(bodyId);
            B2.Rot rotation = B2.BodyGetRotation(bodyId);

            Console.WriteLine($"Position: ({position.x}, {position.y})");
            Console.WriteLine($"Angle: {MathF.Atan2(rotation.s, rotation.c)}");
            Console.WriteLine();
        }
    }
}
