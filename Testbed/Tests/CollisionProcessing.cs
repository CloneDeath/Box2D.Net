﻿using System;
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
				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-50.0f, 0.0f), new b2Vec2(50.0f, 0.0f));

				b2FixtureDef sd;
				sd.shape = shape;;

				b2BodyDef bd = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(sd);
			}

			float xLo = -5.0f, xHi = 5.0f;
			float yLo = 2.0f, yHi = 35.0f;

			// Small triangle
			b2Vec2[] vertices = new b2Vec2[3];
			vertices[0].Set(-1.0f, 0.0f);
			vertices[1].Set(1.0f, 0.0f);
			vertices[2].Set(0.0f, 2.0f);

			b2PolygonShape polygon;
			polygon.Set(vertices, 3);

			b2FixtureDef triangleShapeDef;
			triangleShapeDef.shape = polygon;
			triangleShapeDef.density = 1.0f;

			b2BodyDef triangleBodyDef;
			triangleBodyDef.type = b2BodyType.b2_dynamicBody;
			triangleBodyDef.position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			b2Body body1 = m_world.CreateBody(triangleBodyDef);
			body1.CreateFixture(triangleShapeDef);

			// Large triangle (recycle definitions)
			vertices[0] *= 2.0f;
			vertices[1] *= 2.0f;
			vertices[2] *= 2.0f;
			polygon.Set(vertices, 3);

			triangleBodyDef.position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			b2Body body2 = m_world.CreateBody(triangleBodyDef);
			body2.CreateFixture(triangleShapeDef);
		
			// Small box
			polygon.SetAsBox(1.0f, 0.5f);

			b2FixtureDef boxShapeDef = new b2FixtureDef();
			boxShapeDef.shape = polygon;
			boxShapeDef.density = 1.0f;

			b2BodyDef boxBodyDef = new b2BodyDef();
			boxBodyDef.type = b2BodyType.b2_dynamicBody;
			boxBodyDef.position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			b2Body body3 = m_world.CreateBody(boxBodyDef);
			body3.CreateFixture(boxShapeDef);

			// Large box (recycle definitions)
			polygon.SetAsBox(2.0f, 1.0f);
			boxBodyDef.position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));
		
			b2Body body4 = m_world.CreateBody(boxBodyDef);
			body4.CreateFixture(boxShapeDef);

			// Small circle
			b2CircleShape circle = new b2CircleShape();
			circle.m_radius = 1.0f;

			b2FixtureDef circleShapeDef = new b2FixtureDef();
			circleShapeDef.shape = circle;
			circleShapeDef.density = 1.0f;

			b2BodyDef circleBodyDef = new b2BodyDef();
			circleBodyDef.type = b2BodyType.b2_dynamicBody;
			circleBodyDef.position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			b2Body body5 = m_world.CreateBody(circleBodyDef);
			body5.CreateFixture(circleShapeDef);

			// Large circle
			circle.m_radius *= 2.0f;
			circleBodyDef.position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

			b2Body body6 = m_world.CreateBody(circleBodyDef);
			body6.CreateFixture(circleShapeDef);
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			// We are going to destroy some bodies according to contact
			// points. We must buffer the bodies that should be destroyed
			// because they may belong to multiple contact points.
			const int k_maxNuke = 6;
			b2Body[] nuke = new b2Body[k_maxNuke];
			int nukeCount = 0;

			// Traverse the contact results. Destroy bodies that
			// are touching heavier bodies.
			for (int i = 0; i < m_pointCount; ++i)
			{
				ContactPoint* point = m_points + i;

				b2Body body1 = point.fixtureA.GetBody();
				b2Body body2 = point.fixtureB.GetBody();
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
			std::sort(nuke, nuke + nukeCount);

			// Destroy the bodies, skipping duplicates.
			int i = 0;
			while (i < nukeCount)
			{
				b2Body b = nuke[i++];
				while (i < nukeCount && nuke[i] == b)
				{
					++i;
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
