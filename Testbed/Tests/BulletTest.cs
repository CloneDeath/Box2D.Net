using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class BulletTest : Test
	{
		Body m_body;
		Body m_bullet;
		float m_x;

		public BulletTest()
		{
			{
				BodyDef bd = new BodyDef();
				bd.position.Set(0.0f, 0.0f);
				Body body = m_world.CreateBody(bd);

				EdgeShape edge = new EdgeShape();

				edge.Set(new Vec2(-10.0f, 0.0f), new Vec2(10.0f, 0.0f));
				body.CreateFixture(edge, 0.0f);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.2f, 1.0f, new Vec2(0.5f, 1.0f), 0.0f);
				body.CreateFixture(shape, 0.0f);
			}

			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(0.0f, 4.0f);

				PolygonShape box = new PolygonShape();
				box.SetAsBox(2.0f, 0.1f);

				m_body = m_world.CreateBody(bd);
				m_body.CreateFixture(box, 1.0f);

				box.SetAsBox(0.25f, 0.25f);

				//m_x = RandomFloat(-1.0f, 1.0f);
				m_x = 0.20352793f;
				bd.position.Set(m_x, 10.0f);
				bd.bullet = true;

				m_bullet = m_world.CreateBody(bd);
				m_bullet.CreateFixture(box, 100.0f);

				m_bullet.SetLinearVelocity(new Vec2(0.0f, -50.0f));
			}
		}

		void Launch()
		{
			m_body.SetTransform(new Vec2(0.0f, 4.0f), 0.0f);
			m_body.SetLinearVelocity(new Vec2(0, 0));
			m_body.SetAngularVelocity(0.0f);

			m_x = RandomFloat(-1.0f, 1.0f);
			m_bullet.SetTransform(new Vec2(m_x, 10.0f), 0.0f);
			m_bullet.SetLinearVelocity(new Vec2(0.0f, -50.0f));
			m_bullet.SetAngularVelocity(0.0f);


			Utilities._gjkCalls = 0;
			Utilities._gjkIters = 0;
			Utilities._gjkMaxIters = 0;

			Utilities._toiCalls = 0;
			Utilities._toiIters = 0;
			Utilities._toiMaxIters = 0;
			Utilities._toiRootIters = 0;
			Utilities._toiMaxRootIters = 0;
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			if (Utilities._gjkCalls > 0)
			{
				m_debugDraw.DrawString("gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
					Utilities._gjkCalls, Utilities._gjkIters / (float)Utilities._gjkCalls, Utilities._gjkMaxIters);
				
			}

			if (Utilities._toiCalls > 0)
			{
				m_debugDraw.DrawString("toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
					Utilities._toiCalls, Utilities._toiIters / (float)Utilities._toiCalls, Utilities._toiMaxRootIters);
				

				m_debugDraw.DrawString("ave toi root iters = %3.1f, max toi root iters = %d",
					Utilities._toiRootIters / (float)(Utilities._toiCalls), Utilities._toiMaxRootIters);
				
			}

			if (m_stepCount % 60 == 0)
			{
				Launch();
			}
		}

		public static Test Create()
		{
			return new BulletTest();
		}
	}
}
