﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace HelloWorld {
	class HelloWorld {
		// This is a simple example of building and running a simulation
		// using Box2D. Here we create a large ground box and a small dynamic
		// box.
		// There are no graphics for this example. Box2D is meant to be used
		// with your rendering engine in your game engine.
		public static int Main(string[] args)
		{
			// Define the gravity vector.
			Vec2 gravity = new Vec2(0.0f, -10.0f);

			// Construct a world object, which will hold and simulate the rigid bodies.
			World world = new World(gravity);

			// Define the ground body.
			BodyDef groundBodyDef = new BodyDef();
			groundBodyDef.Position.Set(0.0f, -10.0f);

			// Call the body factory which allocates memory for the ground body
			// from a pool and creates the ground box shape (also from a pool).
			// The body is also added to the world.
			Body groundBody = world.CreateBody(groundBodyDef);

			// Define the ground box shape.
			PolygonShape groundBox = new PolygonShape();

			// The extents are the half-widths of the box.
			groundBox.SetAsBox(50.0f, 10.0f);
			groundBox.Density = 0;

			// Add the ground fixture to the ground body.
			groundBody.CreateFixture(groundBox);

			// Define the dynamic body. We set its position and call the body factory.
			BodyDef bodyDef = new BodyDef();
			bodyDef.type = BodyType._dynamicBody;
			bodyDef.Position = new Vec2(0.0f, 4.0f);
			Body body = world.CreateBody(bodyDef);

			// Define another box shape for our dynamic body.
			PolygonShape dynamicBox = new PolygonShape();
			dynamicBox.SetAsBox(1.0f, 1.0f);

			// Define the dynamic body fixture.
			FixtureDef fixtureDef = new FixtureDef();
			fixtureDef.shape = dynamicBox;

			// Set the box Density to be non-zero, so it will be dynamic.
			fixtureDef.Density = 1.0f;

			// Override the default friction.
			fixtureDef.friction = 0.3f;

			// Add the shape to the body.
			body.CreateFixture(fixtureDef);

			// Prepare for simulation. Typically we use a time step of 1/60 of a
			// second (60Hz) and 10 iterations. This provides a high quality simulation
			// in most game scenarios.
			float timeStep = 1.0f / 60.0f;
			int velocityIterations = 6;
			int positionIterations = 2;

			// This is our little game loop.
			for (int i = 0; i < 60; ++i)
			{
				// Instruct the world to perform a single step of simulation.
				// It is generally best to keep the time step and iterations fixed.
				world.Step(timeStep, velocityIterations, positionIterations);

				// Now print the position and angle of the body.
				Vec2 position = body.GetPosition();
				float angle = body.GetAngle();

				Console.WriteLine("{0} {1} {2}", position.X.ToString("0000.00"), position.Y.ToString("0000.00"), angle.ToString("0000.00"));
			}

			// When the world destructor is called, all bodies and joints are freed. This can
			// create orphaned pointers, so be careful about your world management.
			Console.WriteLine("Done!");
			Console.ReadLine();
			return 0;
		}
	}
}
