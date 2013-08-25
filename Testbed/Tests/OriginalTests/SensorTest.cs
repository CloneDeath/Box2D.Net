using System;
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
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				{
					EdgeShape shape = new EdgeShape();
					shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
					shape.Density = 0;
					ground.CreateFixture(shape);
				}

	#if ZERO
				{
					FixtureDef sd;
					sd.SetAsBox(10.0f, 2.0f, new Vec2(0.0f, 20.0f), 0.0f);
					sd.IsSensor = true;
					m_sensor = ground.CreateFixture(sd);
				}
	#else
				{
					CircleShape shape = new CircleShape();
					shape.m_radius = 5.0f;
					shape.m_p.Set(0.0f, 10.0f);

					FixtureDef fd = new FixtureDef();
					fd.shape = shape;
					fd.IsSensor = true;
					m_sensor = ground.CreateFixture(fd);
				}
	#endif
			}

			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 1.0f;

				for (int i = 0; i < e_count; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-10.0f + 3.0f * i, 20.0f);
					bd.UserData = m_touching[i];

					m_touching[i] = false;
					m_bodies[i] = m_world.CreateBody(bd);

					m_bodies[i].CreateFixture(shape);
				}
			}
		}

		// Implement contact listener.
		public void BeginContact(Contact contact)
		{
			Fixture fixtureA = contact.FixtureA;
			Fixture fixtureB = contact.FixtureB;

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
		public void EndContact(Contact contact)
		{
			Fixture fixtureA = contact.FixtureA;
			Fixture fixtureB = contact.FixtureB;

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

		public override void Step(TestSettings settings)
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

				Body body = m_bodies[i];
				Body ground = m_sensor.GetBody();

				CircleShape circle = (CircleShape)m_sensor.GetShape();
				Vec2 center = ground.GetWorldPoint(circle.m_p);

				Vec2 position = body.GetPosition();

				Vec2 d = center - position;
				if (d.LengthSquared() < Single.Epsilon * Single.Epsilon)
				{
					continue;
				}

				d.Normalize();
				Vec2 F = 100.0f * d;
				body.ApplyForce(F, position, false);
			}
		}

		public static Test Create()
		{
			return new SensorTest();
		}

		Fixture m_sensor;
		Body[] m_bodies = new Body[e_count];
		bool[] m_touching = new bool[e_count];
	};
}
