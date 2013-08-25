﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// This is used to test sensor shapes.
	class SensorTest : Test
	{
		const int e_count = 7;

		public SensorTest()
		{
			{
				b2BodyDef bd = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd);

				{
					b2EdgeShape shape = new b2EdgeShape();
					shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
					ground.CreateFixture(shape, 0.0f);
				}

	#if ZERO
				{
					b2FixtureDef sd;
					sd.SetAsBox(10.0f, 2.0f, new b2Vec2(0.0f, 20.0f), 0.0f);
					sd.isSensor = true;
					m_sensor = ground.CreateFixture(sd);
				}
	#else
				{
					b2CircleShape shape = new b2CircleShape();
					shape.m_radius = 5.0f;
					shape.m_p.Set(0.0f, 10.0f);

					b2FixtureDef fd = new b2FixtureDef();
					fd.shape = shape;
					fd.isSensor = true;
					m_sensor = ground.CreateFixture(fd);
				}
	#endif
			}

			{
				b2CircleShape shape = new b2CircleShape();
				shape.m_radius = 1.0f;

				for (int i = 0; i < e_count; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(-10.0f + 3.0f * i, 20.0f);
					bd.userData = m_touching[i];

					m_touching[i] = false;
					m_bodies[i] = m_world.CreateBody(bd);

					m_bodies[i].CreateFixture(shape, 1.0f);
				}
			}
		}

		// Implement contact listener.
		public void BeginContact(b2Contact contact)
		{
			b2Fixture fixtureA = contact.GetFixtureA();
			b2Fixture fixtureB = contact.GetFixtureB();

			if (fixtureA == m_sensor)
			{
				object userData = fixtureB.GetBody().UserData;
				if (userData != null)
				{
					userData = true;
				}
			}

			if (fixtureB == m_sensor)
			{
				object userData = fixtureA.GetBody().UserData;
				if (userData != null)
				{
					userData = true;
				}
			}
		}

		// Implement contact listener.
		public void EndContact(b2Contact contact)
		{
			b2Fixture fixtureA = contact.GetFixtureA();
			b2Fixture fixtureB = contact.GetFixtureB();

			if (fixtureA == m_sensor)
			{
				if (fixtureB.GetBody().UserData != null)
				{
					fixtureB.GetBody().UserData = false;
				}
			}

			if (fixtureB == m_sensor)
			{
				if (fixtureA.GetBody().UserData != null)
				{
					fixtureA.GetBody().UserData = false;
				}
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			// Traverse the contact results. Apply a force on shapes
			// that overlap the sensor.
			for (int i = 0; i < e_count; ++i)
			{
				if (m_touching[i] == false)
				{
					continue;
				}

				b2Body body = m_bodies[i];
				b2Body ground = m_sensor.GetBody();

				b2CircleShape circle = (b2CircleShape)m_sensor.GetShape();
				b2Vec2 center = ground.GetWorldPoint(circle.m_p);

				b2Vec2 position = body.GetPosition();

				b2Vec2 d = center - position;
				if (d.LengthSquared() < Single.Epsilon * Single.Epsilon)
				{
					continue;
				}

				d.Normalize();
				b2Vec2 F = 100.0f * d;
				body.ApplyForce(F, position, false);
			}
		}

		public static Test Create()
		{
			return new SensorTest();
		}

		b2Fixture m_sensor;
		b2Body[] m_bodies = new b2Body[e_count];
		bool[] m_touching = new bool[e_count];
	};
}
