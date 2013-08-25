using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class Confined : Test
	{
		const int e_columnCount = 0;
		const int e_rowCount = 0;

		public Confined()
		{
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape2 = new EdgeShape();

				// Floor
				shape2.Set(new Vec2(-10.0f, 0.0f), new Vec2(10.0f, 0.0f));
				shape2.Density = 0;
				ground.CreateFixture(shape2);

				// Left wall
				shape2.Set(new Vec2(-10.0f, 0.0f), new Vec2(-10.0f, 20.0f));
				ground.CreateFixture(shape2);

				// Right wall
				shape2.Set(new Vec2(10.0f, 0.0f), new Vec2(10.0f, 20.0f));
				ground.CreateFixture(shape2);

				// Roof
				shape2.Set(new Vec2(-10.0f, 20.0f), new Vec2(10.0f, 20.0f));
				ground.CreateFixture(shape2);
			}

			float radius = 0.5f;
			CircleShape shape = new CircleShape();
			shape.m_p.SetZero();
			shape.m_radius = radius;

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.Density = 1.0f;
			fd.friction = 0.1f;

			for (int j = 0; j < e_columnCount; ++j)
			{
				for (int i = 0; i < e_rowCount; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
					Body body = m_world.CreateBody(bd);

					body.CreateFixture(fd);
				}
			}

			m_world.SetGravity(new Vec2(0.0f, 0.0f));
		}

		void CreateCircle()
		{
			float radius = 2.0f;
			CircleShape shape = new CircleShape();
			shape.m_p.SetZero();
			shape.m_radius = radius;

			FixtureDef fd = new FixtureDef();
			fd.shape = shape;
			fd.Density = 1.0f;
			fd.friction = 0.0f;

			Vec2 p = new Vec2(RandomFloat(), 3.0f + RandomFloat());
			BodyDef bd = new BodyDef();
			bd.type = BodyType._dynamicBody;
			bd.Position = p;
			//bd.allowSleep = false;
			Body body = m_world.CreateBody(bd);

			body.CreateFixture(fd);
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.C)) {
				CreateCircle();
			}
		}

		public override void Step(TestSettings settings)
		{
			bool sleeping = true;
			foreach (Body b in m_world.GetBodyList())
			{
				if (b.GetBodyType() != BodyType._dynamicBody)
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

			foreach (Body b in m_world.GetBodyList())
			{
				if (b.GetBodyType() != BodyType._dynamicBody)
				{
					continue;
				}

				Vec2 p = b.GetPosition();
				if (p.X <= -10.0f || 10.0f <= p.X || p.Y <= 0.0f || 20.0f <= p.Y)
				{
					p.X += 0.0f;
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
