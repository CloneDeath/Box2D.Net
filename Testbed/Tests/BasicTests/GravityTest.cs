using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests.BasicTests {
	class GravityTest : Test {
		public GravityTest() {
			// Define the gravity vector.
			Vec2 gravity = new Vec2(0.0f, 10.0f);

			// Construct a world object, which will hold and simulate the rigid bodies.
			m_world.SetGravity(gravity);

			// Define the ground body.
			BodyDef groundBodyDef = new BodyDef();
			groundBodyDef.Position.Set(0.0f, 20.0f);

			// Call the body factory which allocates memory for the ground body
			// from a pool and creates the ground box shape (also from a pool).
			// The body is also added to the world.
			Body groundBody = m_world.CreateBody(groundBodyDef);

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
			Body body = m_world.CreateBody(bodyDef);

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
		}

		public static Test Create() {
			return new GravityTest();
		}
	}
}
