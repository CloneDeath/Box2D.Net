using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class OneSidedPlatform : Test
	{
		public enum State
		{
			e_unknown,
			e_above,
			e_below
		};

		public OneSidedPlatform()
		{
			// Ground
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Platform
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, 10.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(3.0f, 0.5f);
				shape.Density = 0;
				m_platform = body.CreateFixture(shape);

				m_bottom = 10.0f - 0.5f;
				m_top = 10.0f + 0.5f;
			}

			// Actor
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(0.0f, 12.0f);
				Body body = m_world.CreateBody(bd);

				m_radius = 0.5f;
				CircleShape shape = new CircleShape();
				shape.m_radius = m_radius;
				shape.Density = 20;
				m_character = body.CreateFixture(shape);

				body.SetLinearVelocity(new Vec2(0.0f, -50.0f));

				m_state = State.e_unknown;
			}
		}

		public override void PreSolve(Contact contact, Manifold oldManifold)
		{
			base.PreSolve(contact, oldManifold);

			Fixture fixtureA = contact.FixtureA;
			Fixture fixtureB = contact.FixtureB;

			if (fixtureA != m_platform && fixtureA != m_character)
			{
				return;
			}

			if (fixtureB != m_platform && fixtureB != m_character)
			{
				return;
			}

	#if true
			Vec2 position = m_character.GetBody().GetPosition();

			if (position.Y < m_top + m_radius - 3.0f *Settings._linearSlop)
			{
				contact.SetEnabled(false);
			}
	#else
			Vec2 v = m_character.GetBody().GetLinearVelocity();
			if (v.Y > 0.0f)
			{
				contact.SetEnabled(false);
			}
	#endif
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Press: (c) create a shape, (d) destroy a shape.");
			

			Vec2 v = m_character.GetBody().GetLinearVelocity();
			m_debugDraw.DrawString("Character Linear Velocity: %f", v.Y);
			
		}

		public static Test Create()
		{
			return new OneSidedPlatform();
		}

		float m_radius, m_top, m_bottom;
		State m_state;
		Fixture m_platform;
		Fixture m_character;
	};
}
