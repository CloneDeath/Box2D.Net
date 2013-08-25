using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class OneSidedPlatform : Test
	{
	public:

		enum State
		{
			e_unknown,
			e_above,
			e_below
		};

		OneSidedPlatform()
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
				bd.position.Set(0.0f, 10.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2PolygonShape shape;
				shape.SetAsBox(3.0f, 0.5f);
				m_platform = body.CreateFixture(&shape, 0.0f);

				m_bottom = 10.0f - 0.5f;
				m_top = 10.0f + 0.5f;
			}

			// Actor
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 12.0f);
				b2Body* body = m_world.CreateBody(&bd);

				m_radius = 0.5f;
				b2CircleShape shape;
				shape.m_radius = m_radius;
				m_character = body.CreateFixture(&shape, 20.0f);

				body.SetLinearVelocity(b2Vec2(0.0f, -50.0f));

				m_state = e_unknown;
			}
		}

		void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
		{
			Test::PreSolve(contact, oldManifold);

			b2Fixture* fixtureA = contact.GetFixtureA();
			b2Fixture* fixtureB = contact.GetFixtureB();

			if (fixtureA != m_platform && fixtureA != m_character)
			{
				return;
			}

			if (fixtureB != m_platform && fixtureB != m_character)
			{
				return;
			}

	#if 1
			b2Vec2 position = m_character.GetBody().GetPosition();

			if (position.y < m_top + m_radius - 3.0f *b2Settings.b2_linearSlop)
			{
				contact.SetEnabled(false);
			}
	#else
			b2Vec2 v = m_character.GetBody().GetLinearVelocity();
			if (v.y > 0.0f)
			{
				contact.SetEnabled(false);
			}
	#endif
		}

		void Step(Settings* settings)
		{
			Test::Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
			m_textLine += DRAW_STRING_NEW_LINE;

			b2Vec2 v = m_character.GetBody().GetLinearVelocity();
			m_debugDraw.DrawString(5, m_textLine, "Character Linear Velocity: %f", v.y);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		static Test* Create()
		{
			return new OneSidedPlatform;
		}

		float m_radius, m_top, m_bottom;
		State m_state;
		b2Fixture* m_platform;
		b2Fixture* m_character;
	};
}
