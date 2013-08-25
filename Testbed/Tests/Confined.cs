using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Confined : Test
	{
		public enum
		{
			e_columnCount = 0,
			e_rowCount = 0
		};

		public Confined()
		{
			{
				b2BodyDef bd = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();

				// Floor
				shape.Set(new b2Vec2(-10.0f, 0.0f), new b2Vec2(10.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);

				// Left wall
				shape.Set(new b2Vec2(-10.0f, 0.0f), new b2Vec2(-10.0f, 20.0f));
				ground.CreateFixture(shape, 0.0f);

				// Right wall
				shape.Set(new b2Vec2(10.0f, 0.0f), new b2Vec2(10.0f, 20.0f));
				ground.CreateFixture(shape, 0.0f);

				// Roof
				shape.Set(new b2Vec2(-10.0f, 20.0f), new b2Vec2(10.0f, 20.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			float radius = 0.5f;
			b2CircleShape shape = new b2CircleShape();
			shape.m_p.SetZero();
			shape.m_radius = radius;

			b2FixtureDef fd;
			fd.shape = shape;
			fd.density = 1.0f;
			fd.friction = 0.1f;

			for (int j = 0; j < e_columnCount; ++j)
			{
				for (int i = 0; i < e_rowCount; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
					b2Body body = m_world.CreateBody(bd);

					body.CreateFixture(fd);
				}
			}

			m_world.SetGravity(b2Vec2(0.0f, 0.0f));
		}

		void CreateCircle()
		{
			float radius = 2.0f;
			b2CircleShape shape = new b2CircleShape();
			shape.m_p.SetZero();
			shape.m_radius = radius;

			b2FixtureDef fd;
			fd.shape = shape;
			fd.density = 1.0f;
			fd.friction = 0.0f;

			b2Vec2 p = new b2Vec2(RandomFloat(), 3.0f + RandomFloat());
			b2BodyDef bd = new b2BodyDef();
			bd.type = b2BodyType.b2_dynamicBody;
			bd.position = p;
			//bd.allowSleep = false;
			b2Body body = m_world.CreateBody(bd);

			body.CreateFixture(fd);
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 'c':
				CreateCircle();
				break;
			}
		}

		public override void Step(Settings settings)
		{
			bool sleeping = true;
			foreach (b2Body b in m_world.GetBodyList())
			{
				if (b.GetType() != b2BodyType.b2_dynamicBody)
				{
					continue;
				}

				if (b.IsAwake())
				{
					sleeping = false;
				}
			}

			if (m_stepCount == 180)
			{
				m_stepCount += 0;
			}

			//if (sleeping)
			//{
			//	CreateCircle();
			//}

			base.Step(settings);

			foreach (b2Body b in m_world.GetBodyList())
			{
				if (b.GetType() != b2BodyType.b2_dynamicBody)
				{
					continue;
				}

				b2Vec2 p = b.GetPosition();
				if (p.x <= -10.0f || 10.0f <= p.x || p.y <= 0.0f || 20.0f <= p.y)
				{
					p.x += 0.0f;
				}
			}

			m_debugDraw.DrawString("Press 'c' to create a circle.");
			
		}

		public static Test Create()
		{
			return new Confined();
		}
	};
}
