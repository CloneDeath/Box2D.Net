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

	#if true
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(0.0f, 20.0f);
				//bd.angle = 0.1f;

				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(2.0f, 0.1f);

				m_body = m_world.CreateBody(bd);
				m_body.CreateFixture(shape, 1.0f);

				m_angularVelocity = RandomFloat(-50.0f, 50.0f);
				//m_angularVelocity = 46.661274f;
				m_body.SetLinearVelocity(new b2Vec2(0.0f, -100.0f));
				m_body.SetAngularVelocity(m_angularVelocity);
			}
	#else
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(0.0f, 2.0f);
				b2Body body = m_world.CreateBody(bd);

				b2CircleShape shape = new b2CircleShape();
				shape.m_p.SetZero();
				shape.m_radius = 0.5f;
				body.CreateFixture(shape, 1.0f);

				bd.bullet = true;
				bd.position.Set(0.0f, 10.0f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(shape, 1.0f);
				body.SetLinearVelocity(new b2Vec2(0.0f, -100.0f));
			}
	#endif

			b2Distance.b2_gjkCalls = 0; b2Distance.b2_gjkIters = 0; b2Distance.b2_gjkMaxIters = 0;
			b2TimeOfImpact.b2_toiCalls = 0; b2TimeOfImpact.b2_toiIters = 0;
			b2TimeOfImpact.b2_toiRootIters = 0; b2TimeOfImpact.b2_toiMaxRootIters = 0;
			b2TimeOfImpact.b2_toiTime = 0.0f; b2TimeOfImpact.b2_toiMaxTime = 0.0f;
		}

		public void Launch()
		{

			b2Distance.b2_gjkCalls = 0; b2Distance.b2_gjkIters = 0; b2Distance.b2_gjkMaxIters = 0;
			b2TimeOfImpact.b2_toiCalls = 0; b2TimeOfImpact.b2_toiIters = 0;
			b2TimeOfImpact.b2_toiRootIters = 0; b2TimeOfImpact.b2_toiMaxRootIters = 0;
			b2TimeOfImpact.b2_toiTime = 0.0f; b2TimeOfImpact.b2_toiMaxTime = 0.0f;

			m_body.SetTransform(new b2Vec2(0.0f, 20.0f), 0.0f);
			m_angularVelocity = RandomFloat(-50.0f, 50.0f);
			m_body.SetLinearVelocity(new b2Vec2(0.0f, -100.0f));
			m_body.SetAngularVelocity(m_angularVelocity);
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			if (b2Distance.b2_gjkCalls > 0)
			{
				m_debugDraw.DrawString("gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
					b2Distance.b2_gjkCalls, b2Distance.b2_gjkIters / (float)(b2Distance.b2_gjkCalls), b2Distance.b2_gjkMaxIters);
				
			}

			if (b2TimeOfImpact.b2_toiCalls > 0)
			{
				m_debugDraw.DrawString("toi calls = %d, ave [max] toi iters = %3.1f [%d]",
									b2TimeOfImpact.b2_toiCalls, b2TimeOfImpact.b2_toiIters / (float)(b2TimeOfImpact.b2_toiCalls), b2TimeOfImpact.b2_toiMaxRootIters);
				
			
				m_debugDraw.DrawString("ave [max] toi root iters = %3.1f [%d]",
					b2TimeOfImpact.b2_toiRootIters / (float)(b2TimeOfImpact.b2_toiCalls), b2TimeOfImpact.b2_toiMaxRootIters);
				

				m_debugDraw.DrawString("ave [max] toi time = %.1f [%.1f] (microseconds)",
					1000.0f * b2TimeOfImpact.b2_toiTime / (float)(b2TimeOfImpact.b2_toiCalls), 1000.0f * b2TimeOfImpact.b2_toiMaxTime);
				
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

		b2Body m_body;
		float m_angularVelocity;
	};
}
