﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// This is used to test sensor shapes.
	class Breakable : Test
	{
	public enum
		{
			e_count = 7
		};

		Breakable()
		{
			// Ground body
			{
				b2BodyDef bd;
				b2Body ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			// Breakable dynamic body
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 40.0f);
				bd.angle = 0.25f * Math.PI;
				m_body1 = m_world.CreateBody(&bd);

				m_shape1.SetAsBox(0.5f, 0.5f, b2Vec2(-0.5f, 0.0f), 0.0f);
				m_piece1 = m_body1.CreateFixture(&m_shape1, 1.0f);

				m_shape2.SetAsBox(0.5f, 0.5f, b2Vec2(0.5f, 0.0f), 0.0f);
				m_piece2 = m_body1.CreateFixture(&m_shape2, 1.0f);
			}

			m_break = false;
			m_broke = false;
		}

		void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
		{
			if (m_broke)
			{
				// The body already broke.
				return;
			}

			// Should the body break?
			int count = contact.GetManifold().pointCount;

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
			b2Body body1 = m_piece1.GetBody();
			b2Vec2 center = body1.GetWorldCenter();

			body1.DestroyFixture(m_piece2);
			m_piece2 = null;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = body1.GetPosition();
			bd.angle = body1.GetAngle();

			b2Body body2 = m_world.CreateBody(&bd);
			m_piece2 = body2.CreateFixture(&m_shape2, 1.0f);

			// Compute consistent velocities for new bodies based on
			// cached velocity.
			b2Vec2 center1 = body1.GetWorldCenter();
			b2Vec2 center2 = body2.GetWorldCenter();
		
			b2Vec2 velocity1 = m_velocity + Utilities.b2Cross(m_angularVelocity, center1 - center);
			b2Vec2 velocity2 = m_velocity + Utilities.b2Cross(m_angularVelocity, center2 - center);

			body1.SetAngularVelocity(m_angularVelocity);
			body1.SetLinearVelocity(velocity1);

			body2.SetAngularVelocity(m_angularVelocity);
			body2.SetLinearVelocity(velocity2);
		}

		void Step(Settings* settings)
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

		b2Body m_body1;
		b2Vec2 m_velocity;
		float m_angularVelocity;
		b2PolygonShape m_shape1;
		b2PolygonShape m_shape2;
		b2Fixture* m_piece1;
		b2Fixture* m_piece2;

		bool m_broke;
		bool m_break;
	};
}
