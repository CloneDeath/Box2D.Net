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
			b2Vec2 gravity = new b2Vec2(0.0f, -10.0f);

			// Construct a world object, which will hold and simulate the rigid bodies.
			b2World world = new b2World(gravity);

			// Define the ground body.
			b2BodyDef groundBodyDef = new b2BodyDef();
			groundBodyDef.position.Set(0.0f, -10.0f);

			// Call the body factory which allocates memory for the ground body
			// from a pool and creates the ground box shape (also from a pool).
			// The body is also added to the world.
			b2Body groundBody = world.CreateBody(groundBodyDef);

			// Define the ground box shape.
			b2PolygonShape groundBox = new b2PolygonShape();

			// The extents are the half-widths of the box.
			groundBox.SetAsBox(50.0f, 10.0f);

			// Add the ground fixture to the ground body.
			groundBody.CreateFixture(groundBox, 0.0f);

			// Define the dynamic body. We set its position and call the body factory.
			b2BodyDef bodyDef = new b2BodyDef();
			bodyDef.type = b2BodyType.b2_dynamicBody;
			bodyDef.position = new b2Vec2(0.0f, 4.0f);
			b2Body body = world.CreateBody(bodyDef);

			// Define another box shape for our dynamic body.
			b2PolygonShape dynamicBox = new b2PolygonShape();
			dynamicBox.SetAsBox(1.0f, 1.0f);

			// Define the dynamic body fixture.
			b2FixtureDef fixtureDef = new b2FixtureDef();
			fixtureDef.shape = dynamicBox;

			// Set the box density to be non-zero, so it will be dynamic.
			fixtureDef.density = 1.0f;

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
				b2Vec2 position = body.GetPosition();
				float angle = body.GetAngle();

				Console.WriteLine("{0} {1} {2}", position.x.ToString("0000.00"), position.y.ToString("0000.00"), angle.ToString("0000.00"));
			}

			// When the world destructor is called, all bodies and joints are freed. This can
			// create orphaned pointers, so be careful about your world management.

			return 0;
		}
	}
}
