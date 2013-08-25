﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class ConveyorBelt : Test
	{
	public:

		ConveyorBelt()
		{
			// Ground
			{
				b2BodyDef bd;
				b2Body* ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			// Platform
			{
				b2BodyDef bd;
				bd.position.Set(-5.0f, 5.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2PolygonShape shape;
				shape.SetAsBox(10.0f, 0.5f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.friction = 0.8f;
				m_platform = body.CreateFixture(&fd);
			}

			// Boxes
			for (int i = 0; i < 5; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-10.0f + 2.0f * i, 7.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2PolygonShape shape;
				shape.SetAsBox(0.5f, 0.5f);
				body.CreateFixture(&shape, 20.0f);
			}
		}

		void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
		{
			Test::PreSolve(contact, oldManifold);

			b2Fixture* fixtureA = contact.GetFixtureA();
			b2Fixture* fixtureB = contact.GetFixtureB();

			if (fixtureA == m_platform)
			{
				contact.SetTangentSpeed(5.0f);
			}

			if (fixtureB == m_platform)
			{
				contact.SetTangentSpeed(-5.0f);
			}
		}

		void Step(Settings* settings)
		{
			Test::Step(settings);
		}

		static Test* Create()
		{
			return new ConveyorBelt;
		}

		b2Fixture* m_platform;
	};
}