using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class VerticalStack : Test
	{
		const int e_columnCount = 5;
		const int e_rowCount = 16;
			//e_columnCount = 1,
			//e_rowCount = 1

		public VerticalStack()
		{
			{
				b2BodyDef bd;
				b2Body ground = m_world.CreateBody(bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);

				shape.Set(b2Vec2(20.0f, 0.0f), b2Vec2(20.0f, 20.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			float xs[5] = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};

			for (int j = 0; j < e_columnCount; ++j)
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.5f, 0.5f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 1.0f;
				fd.friction = 0.3f;

				for (int i = 0; i < e_rowCount; ++i)
				{
					b2BodyDef bd;
					bd.type = b2BodyType.b2_dynamicBody;

					int n = j * e_rowCount + i;
					Utilities.Assert(n < e_rowCount * e_columnCount);
					m_indices[n] = n;
					bd.userData = m_indices + n;

					float x = 0.0f;
					//float x = RandomFloat(-0.02f, 0.02f);
					//float x = i % 2 == 0 ? -0.025f : 0.025f;
					bd.position.Set(xs[j] + x, 0.752f + 1.54f * i);
					b2Body body = m_world.CreateBody(bd);

					m_bodies[n] = body;

					body.CreateFixture(&fd);
				}
			}

			m_bullet = null;
		}

		public void Keyboard()
		{
			switch (key)
			{
			case ',':
				if (m_bullet != null)
				{
					m_world.DestroyBody(m_bullet);
					m_bullet = null;
				}

				{
					b2CircleShape shape;
					shape.m_radius = 0.25f;

					b2FixtureDef fd;
					fd.shape = &shape;
					fd.density = 20.0f;
					fd.restitution = 0.05f;

					b2BodyDef bd;
					bd.type = b2BodyType.b2_dynamicBody;
					bd.bullet = true;
					bd.position.Set(-31.0f, 5.0f);

					m_bullet = m_world.CreateBody(bd);
					m_bullet.CreateFixture(&fd);

					m_bullet.SetLinearVelocity(b2Vec2(400.0f, 0.0f));
				}
				break;
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Press: (,) to launch a bullet.");
			m_textLine += DRAW_STRING_NEW_LINE;

			//if (m_stepCount == 300)
			//{
			//	if (m_bullet != null)
			//	{
			//		m_world.DestroyBody(m_bullet);
			//		m_bullet = null;
			//	}

			//	{
			//		b2CircleShape shape;
			//		shape.m_radius = 0.25f;

			//		b2FixtureDef fd;
			//		fd.shape = &shape;
			//		fd.density = 20.0f;
			//		fd.restitution = 0.05f;

			//		b2BodyDef bd;
			//		bd.type = b2BodyType.b2_dynamicBody;
			//		bd.bullet = true;
			//		bd.position.Set(-31.0f, 5.0f);

			//		m_bullet = m_world.CreateBody(bd);
			//		m_bullet.CreateFixture(&fd);

			//		m_bullet.SetLinearVelocity(b2Vec2(400.0f, 0.0f));
			//	}
			//}
		}

		public static Test Create()
		{
			return new VerticalStack();
		}

		b2Body m_bullet;
		b2Body[] m_bodied = new b2Body[e_rowCount * e_columnCount];
		int m_indices = new int[e_rowCount * e_columnCount];
	};
}
