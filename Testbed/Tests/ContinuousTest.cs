using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class ContinuousTest : Test
	{
		public ContinuousTest()
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

	#if true
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(0.0f, 20.0f);
				//bd.angle = 0.1f;

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(2.0f, 0.1f);

				m_body = m_world.CreateBody(bd);
				m_body.CreateFixture(shape, 1.0f);

				m_angularVelocity = RandomFloat(-50.0f, 50.0f);
				//m_angularVelocity = 46.661274f;
				m_body.SetLinearVelocity(new Vec2(0.0f, -100.0f));
				m_body.SetAngularVelocity(m_angularVelocity);
			}
	#else
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(0.0f, 2.0f);
				Body body = m_world.CreateBody(bd);

				CircleShape shape = new CircleShape();
				shape.m_p.SetZero();
				shape.m_radius = 0.5f;
				body.CreateFixture(shape, 1.0f);

				bd.bullet = true;
				bd.position.Set(0.0f, 10.0f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(shape, 1.0f);
				body.SetLinearVelocity(new Vec2(0.0f, -100.0f));
			}
	#endif

			Utilities._gjkCalls = 0; Utilities._gjkIters = 0; Utilities._gjkMaxIters = 0;
			Utilities._toiCalls = 0; Utilities._toiIters = 0;
			Utilities._toiRootIters = 0; Utilities._toiMaxRootIters = 0;
			Utilities._toiTime = 0.0f; Utilities._toiMaxTime = 0.0f;
		}

		public void Launch()
		{

			Utilities._gjkCalls = 0; Utilities._gjkIters = 0; Utilities._gjkMaxIters = 0;
			Utilities._toiCalls = 0; Utilities._toiIters = 0;
			Utilities._toiRootIters = 0; Utilities._toiMaxRootIters = 0;
			Utilities._toiTime = 0.0f; Utilities._toiMaxTime = 0.0f;

			m_body.SetTransform(new Vec2(0.0f, 20.0f), 0.0f);
			m_angularVelocity = RandomFloat(-50.0f, 50.0f);
			m_body.SetLinearVelocity(new Vec2(0.0f, -100.0f));
			m_body.SetAngularVelocity(m_angularVelocity);
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			if (Utilities._gjkCalls > 0)
			{
				m_debugDraw.DrawString("gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
					Utilities._gjkCalls, Utilities._gjkIters / (float)(Utilities._gjkCalls), Utilities._gjkMaxIters);
				
			}

			if (Utilities._toiCalls > 0)
			{
				m_debugDraw.DrawString("toi calls = %d, ave [max] toi iters = %3.1f [%d]",
									Utilities._toiCalls, Utilities._toiIters / (float)(Utilities._toiCalls), Utilities._toiMaxRootIters);
				
			
				m_debugDraw.DrawString("ave [max] toi root iters = %3.1f [%d]",
					Utilities._toiRootIters / (float)(Utilities._toiCalls), Utilities._toiMaxRootIters);
				

				m_debugDraw.DrawString("ave [max] toi time = %.1f [%.1f] (microseconds)",
					1000.0f * Utilities._toiTime / (float)(Utilities._toiCalls), 1000.0f * Utilities._toiMaxTime);
				
			}

			if (m_stepCount % 60 == 0)
			{
				//Launch();
			}
		}

		public static Test Create()
		{
			return new ContinuousTest();
		}

		Body m_body;
		float m_angularVelocity;
	};
}
