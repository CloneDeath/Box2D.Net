using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class BulletTest : Test
	{
		b2Body m_body;
		b2Body m_bullet;
		float m_x;

		public BulletTest()
		{
			{
				b2BodyDef bd = new b2BodyDef();
				bd.position.Set(0.0f, 0.0f);
				b2Body body = m_world.CreateBody(bd);

				b2EdgeShape edge = new b2EdgeShape();

				edge.Set(new b2Vec2(-10.0f, 0.0f), new b2Vec2(10.0f, 0.0f));
				body.CreateFixture(edge, 0.0f);

				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.2f, 1.0f, new b2Vec2(0.5f, 1.0f), 0.0f);
				body.CreateFixture(shape, 0.0f);
			}

			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(0.0f, 4.0f);

				b2PolygonShape box = new b2PolygonShape();
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

				m_bullet.SetLinearVelocity(new b2Vec2(0.0f, -50.0f));
			}
		}

		void Launch()
		{
			m_body.SetTransform(new b2Vec2(0.0f, 4.0f), 0.0f);
			m_body.SetLinearVelocity(new b2Vec2(0, 0));
			m_body.SetAngularVelocity(0.0f);

			m_x = RandomFloat(-1.0f, 1.0f);
			m_bullet.SetTransform(new b2Vec2(m_x, 10.0f), 0.0f);
			m_bullet.SetLinearVelocity(new b2Vec2(0.0f, -50.0f));
			m_bullet.SetAngularVelocity(0.0f);


			b2Distance.b2_gjkCalls = 0;
			b2Distance.b2_gjkIters = 0;
			b2Distance.b2_gjkMaxIters = 0;

			b2TimeOfImpact.b2_toiCalls = 0;
			b2TimeOfImpact.b2_toiIters = 0;
			b2TimeOfImpact.b2_toiMaxIters = 0;
			b2TimeOfImpact.b2_toiRootIters = 0;
			b2TimeOfImpact.b2_toiMaxRootIters = 0;
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			if (b2Distance.b2_gjkCalls > 0)
			{
				m_debugDraw.DrawString("gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
					b2Distance.b2_gjkCalls, b2Distance.b2_gjkIters / (float)b2Distance.b2_gjkCalls, b2Distance.b2_gjkMaxIters);
				
			}

			if (b2TimeOfImpact.b2_toiCalls > 0)
			{
				m_debugDraw.DrawString("toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
					b2TimeOfImpact.b2_toiCalls, b2TimeOfImpact.b2_toiIters / (float)b2TimeOfImpact.b2_toiCalls, b2TimeOfImpact.b2_toiMaxRootIters);
				

				m_debugDraw.DrawString("ave toi root iters = %3.1f, max toi root iters = %d",
					b2TimeOfImpact.b2_toiRootIters / (float)(b2TimeOfImpact.b2_toiCalls), b2TimeOfImpact.b2_toiMaxRootIters);
				
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
