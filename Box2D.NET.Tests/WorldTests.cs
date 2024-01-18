using System;
using Box2D.NET.Bindings;
using Xunit;

namespace Box2D.NET.Tests
{
    public unsafe class WorldTests
    {
        [Fact]
        public void HelloWorld()
        {
			// Define the gravity vector.
			B2.Vec2 gravity = new B2.Vec2 { x = 0.0f, y = -10.0f };

			// Construct a world object, which will hold and simulate the rigid bodies.
			B2.WorldDef worldDef = B2.defaultWorldDef;
			worldDef.gravity = gravity;

			B2.WorldId worldId = B2.CreateWorld(&worldDef);

			// Define the ground body.
			B2.BodyDef groundBodyDef = B2.defaultBodyDef;
			groundBodyDef.position = new B2.Vec2 { x = 0.0f, y = -10.0f };

			// Call the body factory which allocates memory for the ground body
			// from a pool and creates the ground box shape (also from a pool).
			// The body is also added to the world.
			B2.BodyId groundBodyId = B2.CreateBody(worldId, &groundBodyDef);

			// Define the ground box shape. The extents are the half-widths of the box.
			B2.Polygon groundBox = B2.MakeBox(50.0f, 10.0f);

			// Add the box shape to the ground body.
			B2.ShapeDef groundShapeDef = B2.defaultShapeDef;
			B2.CreatePolygonShape(groundBodyId, &groundShapeDef, &groundBox);

			// Define the dynamic body. We set its position and call the body factory.
			B2.BodyDef bodyDef = B2.defaultBodyDef;
			bodyDef.type = B2.dynamicBody;
			bodyDef.position = new B2.Vec2 { x = 0.0f, y = 4.0f };
			B2.BodyId bodyId = B2.CreateBody(worldId, &bodyDef);

			// Define another box shape for our dynamic body.
			B2.Polygon dynamicBox = B2.MakeBox(1.0f, 1.0f);

			// Define the dynamic body shape
			B2.ShapeDef shapeDef = B2.defaultShapeDef;

			// Set the box density to be non-zero, so it will be dynamic.
			shapeDef.density = 1.0f;

			// Override the default friction.
			shapeDef.friction = 0.3f;

			// Add the shape to the body.
			B2.CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);

			// Prepare for simulation. Typically we use a time step of 1/60 of a
			// second (60Hz) and 10 iterations. This provides a high quality simulation
			// in most game scenarios.
			float timeStep = 1.0f / 60.0f;
			int velocityIterations = 6;
			int relaxIterations = 2;

			B2.Vec2 position = B2.BodyGetPosition(bodyId);
			float angle = B2.BodyGetAngle(bodyId);

			// This is our little game loop.
			for (int i = 0; i < 60; i++)
			{
				// Instruct the world to perform a single step of simulation.
				// It is generally best to keep the time step and iterations fixed.
				B2.WorldStep(worldId, timeStep, velocityIterations, relaxIterations);

				// Now print the position and angle of the body.
				position = B2.BodyGetPosition(bodyId);
				angle = B2.BodyGetAngle(bodyId);

				//printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
			}

			// When the world destructor is called, all bodies and joints are freed. This can
			// create orphaned ids, so be careful about your world management.
			B2.DestroyWorld(worldId);

			Assert.True(Math.Abs(position.x) < 0.01f);
			Assert.True(Math.Abs(position.y - 1.00f) < 0.01f);
			Assert.True(Math.Abs(angle) < 0.01f);
        }
    }
}
