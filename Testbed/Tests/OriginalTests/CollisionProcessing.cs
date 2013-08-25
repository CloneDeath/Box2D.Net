using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// This test shows collision processing and tests
	// deferred body destruction.
	class CollisionProcessing : Test
	{
		public CollisionProcessing()
		{
			// Ground body
			{
				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-50.0f, 0.0f), new Vec2(50.0f, 0.0f));

				FixtureDef sd = new FixtureDef();
				sd.shape = shape;;

				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(sd);
			}

			float xLo = -5.0f, xHi = 5.0f;
			float yLo = 2.0f, yHi = 35.0f;

			// Small triangle
			Vec2[] vertices = new Vec2[3];
			vertices[0].Set(-1.0f, 0.0f);
			vertices[1].Set(1.0f, 0.0f);
			vertices[2].Set(0.0f, 2.0f);

			PolygonShape polygon = new PolygonShape();
			polygon.Set(vertices, 3);

			FixtureDef triangleShapeDef = new FixtureDef();
			triangleShapeDef.shape = polygon;
			triangleShapeDef.Density = 1.0f;

			BodyDef triangleBodyDef = new BodyDef();
			triangleBodyDef.type = BodyType._dynamicBody;
			triangleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			Body body1 = m_world.CreateBody(triangleBodyDef);
			body1.CreateFixture(triangleShapeDef);

			// Large triangle (recycle definitions)
			vertices[0] *= 2.0f;
			vertices[1] *= 2.0f;
			vertices[2] *= 2.0f;
			polygon.Set(vertices, 3);

			triangleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			Body body2 = m_world.CreateBody(triangleBodyDef);
			body2.CreateFixture(triangleShapeDef);
		
			// Small box
			polygon.SetAsBox(1.0f, 0.5f);

			FixtureDef boxShapeDef = new FixtureDef();
			boxShapeDef.shape = polygon;
			boxShapeDef.Density = 1.0f;

			BodyDef boxBodyDef = new BodyDef();
			boxBodyDef.type = BodyType._dynamicBody;
			boxBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			Body body3 = m_world.CreateBody(boxBodyDef);
			body3.CreateFixture(boxShapeDef);

			// Large box (recycle definitions)
			polygon.SetAsBox(2.0f, 1.0f);
			boxBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));
		
			Body body4 = m_world.CreateBody(boxBodyDef);
			body4.CreateFixture(boxShapeDef);

			// Small circle
			CircleShape circle = new CircleShape();
			circle.m_radius = 1.0f;

			FixtureDef circleShapeDef = new FixtureDef();
			circleShapeDef.shape = circle;
			circleShapeDef.Density = 1.0f;

			BodyDef circleBodyDef = new BodyDef();
			circleBodyDef.type = BodyType._dynamicBody;
			circleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			Body body5 = m_world.CreateBody(circleBodyDef);
			body5.CreateFixture(circleShapeDef);

			// Large circle
			circle.m_radius *= 2.0f;
			circleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			Body body6 = m_world.CreateBody(circleBodyDef);
			body6.CreateFixture(circleShapeDef);
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			// We are going to destroy some bodies according to contact
			// points. We must buffer the bodies that should be destroyed
			// because they may belong to multiple contact points.
			const int k_maxNuke = 6;
			Body[] nuke = new Body[k_maxNuke];
			int nukeCount = 0;

			// Traverse the contact results. Destroy bodies that
			// are touching heavier bodies.
			for (int i = 0; i < m_pointCount; ++i)
			{
				ContactPoint point = m_points[i];

				Body body1 = point.fixtureA.GetBody();
				Body body2 = point.fixtureB.GetBody();
				float mass1 = body1.GetMass();
				float mass2 = body2.GetMass();

				if (mass1 > 0.0f && mass2 > 0.0f)
				{
					if (mass2 > mass1)
					{
						nuke[nukeCount++] = body1;
					}
					else
					{
						nuke[nukeCount++] = body2;
					}

					if (nukeCount == k_maxNuke)
					{
						break;
					}
				}
			}

			// Sort the nuke array to group duplicates.
			throw new NotImplementedException();
			//std::sort(nuke, nuke + nukeCount);

			// Destroy the bodies, skipping duplicates.
			int i2 = 0;
			while (i2 < nukeCount)
			{
				Body b = nuke[i2++];
				while (i2 < nukeCount && nuke[i2] == b)
				{
					++i2;
				}

				if (b != m_bomb)
				{
					m_world.DestroyBody(b);
				}
			}
		}

		public static Test Create()
		{
			return new CollisionProcessing();
		}
	};
}
