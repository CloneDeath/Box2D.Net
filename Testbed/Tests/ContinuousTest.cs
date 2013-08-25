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
				b2BodyDef bd;
				bd.position.Set(0.0f, 0.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2EdgeShape edge;

				edge.Set(b2Vec2(-10.0f, 0.0f), b2Vec2(10.0f, 0.0f));
				body.CreateFixture(&edge, 0.0f);

				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 1.0f, b2Vec2(0.5f, 1.0f), 0.0f);
				body.CreateFixture(&shape, 0.0f);
			}

	#if true
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 20.0f);
				//bd.angle = 0.1f;

				b2PolygonShape shape;
				shape.SetAsBox(2.0f, 0.1f);

				m_body = m_world.CreateBody(&bd);
				m_body.CreateFixture(&shape, 1.0f);

				m_angularVelocity = RandomFloat(-50.0f, 50.0f);
				//m_angularVelocity = 46.661274f;
				m_body.SetLinearVelocity(b2Vec2(0.0f, -100.0f));
				m_body.SetAngularVelocity(m_angularVelocity);
			}
	#else
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 2.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2CircleShape shape;
				shape.m_p.SetZero();
				shape.m_radius = 0.5f;
				body.CreateFixture(&shape, 1.0f);

				bd.bullet = true;
				bd.position.Set(0.0f, 10.0f);
				body = m_world.CreateBody(&bd);
				body.CreateFixture(&shape, 1.0f);
				body.SetLinearVelocity(b2Vec2(0.0f, -100.0f));
			}
	#endif

			extern int b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
			extern int b2_toiCalls, b2_toiIters;
			extern int b2_toiRootIters, b2_toiMaxRootIters;
			extern float b2_toiTime, b2_toiMaxTime;

			b2_gjkCalls = 0; b2_gjkIters = 0; b2_gjkMaxIters = 0;
			b2_toiCalls = 0; b2_toiIters = 0;
			b2_toiRootIters = 0; b2_toiMaxRootIters = 0;
			b2_toiTime = 0.0f; b2_toiMaxTime = 0.0f;
		}

		public void Launch()
		{
			extern int b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
			extern int b2_toiCalls, b2_toiIters;
			extern int b2_toiRootIters, b2_toiMaxRootIters;
			extern float b2_toiTime, b2_toiMaxTime;

			b2_gjkCalls = 0; b2_gjkIters = 0; b2_gjkMaxIters = 0;
			b2_toiCalls = 0; b2_toiIters = 0;
			b2_toiRootIters = 0; b2_toiMaxRootIters = 0;
			b2_toiTime = 0.0f; b2_toiMaxTime = 0.0f;

			m_body.SetTransform(b2Vec2(0.0f, 20.0f), 0.0f);
			m_angularVelocity = RandomFloat(-50.0f, 50.0f);
			m_body.SetLinearVelocity(b2Vec2(0.0f, -100.0f));
			m_body.SetAngularVelocity(m_angularVelocity);
		}

		public void Step(Settings* settings)
		{
			Test::Step(settings);

			extern int b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;

			if (b2_gjkCalls > 0)
			{
				m_debugDraw.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
					b2_gjkCalls, b2_gjkIters / float(b2_gjkCalls), b2_gjkMaxIters);
				m_textLine += DRAW_STRING_NEW_LINE;
			}

			extern int b2_toiCalls, b2_toiIters;
			extern int b2_toiRootIters, b2_toiMaxRootIters;
			extern float b2_toiTime, b2_toiMaxTime;

			if (b2_toiCalls > 0)
			{
				m_debugDraw.DrawString(5, m_textLine, "toi calls = %d, ave [max] toi iters = %3.1f [%d]",
									b2_toiCalls, b2_toiIters / float(b2_toiCalls), b2_toiMaxRootIters);
				m_textLine += DRAW_STRING_NEW_LINE;
			
				m_debugDraw.DrawString(5, m_textLine, "ave [max] toi root iters = %3.1f [%d]",
					b2_toiRootIters / float(b2_toiCalls), b2_toiMaxRootIters);
				m_textLine += DRAW_STRING_NEW_LINE;

				m_debugDraw.DrawString(5, m_textLine, "ave [max] toi time = %.1f [%.1f] (microseconds)",
					1000.0f * b2_toiTime / float(b2_toiCalls), 1000.0f * b2_toiMaxTime);
				m_textLine += DRAW_STRING_NEW_LINE;
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

		b2Body* m_body;
		float m_angularVelocity;
	};
}
