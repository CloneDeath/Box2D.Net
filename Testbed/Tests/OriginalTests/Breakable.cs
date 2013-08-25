using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// This is used to test sensor shapes.
	class Breakable : Test
	{
		const int e_count = 7;

		Breakable()
		{
			// Ground body
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Breakable dynamic body
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(0.0f, 40.0f);
				bd.angle = 0.25f * (float)Math.PI;
				m_body1 = m_world.CreateBody(bd);

				m_shape1.SetAsBox(0.5f, 0.5f, new Vec2(-0.5f, 0.0f), 0.0f);
				m_shape1.Density = 1;
				m_piece1 = m_body1.CreateFixture(m_shape1);

				m_shape2.SetAsBox(0.5f, 0.5f, new Vec2(0.5f, 0.0f), 0.0f);
				m_shape2.Density = 1;
				m_piece2 = m_body1.CreateFixture(m_shape2);
			}

			m_break = false;
			m_broke = false;
		}

		void PostSolve(Contact contact, ContactImpulse impulse)
		{
			if (m_broke)
			{
				// The body already broke.
				return;
			}

			// Should the body break?
			int count = contact.GetManifold().points.Count();

			float maxImpulse = 0.0f;
			for (int i = 0; i < count; ++i)
			{
				maxImpulse = Math.Max(maxImpulse, impulse.normalImpulses[i]);
			}

			if (maxImpulse > 40.0f)
			{
				// Flag the body for breaking.
				m_break = true;
			}
		}

		void Break()
		{
			// Create two bodies from one.
			Body body1 = m_piece1.GetBody();
			Vec2 center = body1.GetWorldCenter();

			body1.DestroyFixture(m_piece2);
			m_piece2 = null;

			BodyDef bd = new BodyDef();
			bd.type = BodyType._dynamicBody;
			bd.Position = body1.GetPosition();
			bd.angle = body1.GetAngle();

			Body body2 = m_world.CreateBody(bd);
			m_shape2.Density = 1;
			m_piece2 = body2.CreateFixture(m_shape2);

			// Compute consistent velocities for new bodies based on
			// cached velocity.
			Vec2 center1 = body1.GetWorldCenter();
			Vec2 center2 = body2.GetWorldCenter();
		
			Vec2 velocity1 = m_velocity + Utilities.Cross(m_angularVelocity, center1 - center);
			Vec2 velocity2 = m_velocity + Utilities.Cross(m_angularVelocity, center2 - center);

			body1.SetAngularVelocity(m_angularVelocity);
			body1.SetLinearVelocity(velocity1);

			body2.SetAngularVelocity(m_angularVelocity);
			body2.SetLinearVelocity(velocity2);
		}

		public override void Step(TestSettings settings)
		{
			if (m_break)
			{
				Break();
				m_broke = true;
				m_break = false;
			}

			// Cache velocities to improve movement on breakage.
			if (m_broke == false)
			{
				m_velocity = m_body1.GetLinearVelocity();
				m_angularVelocity = m_body1.GetAngularVelocity();
			}

			base.Step(settings);
		}

		public static Test Create()
		{
			return new Breakable();
		}

		Body m_body1;
		Vec2 m_velocity;
		float m_angularVelocity;
		PolygonShape m_shape1;
		PolygonShape m_shape2;
		Fixture m_piece1;
		Fixture m_piece2;

		bool m_broke;
		bool m_break;
	};
}
