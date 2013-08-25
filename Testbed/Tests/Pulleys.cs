using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Pulleys : Test
	{
		public Pulleys()
		{
			float y = 16.0f;
			float L = 12.0f;
			float a = 1.0f;
			float b = 2.0f;

			b2Body* ground = null;
			{
				b2BodyDef bd;
				ground = m_world.CreateBody(&bd);

				b2EdgeShape edge;
				edge.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				//ground.CreateFixture(&shape, 0.0f);

				b2CircleShape circle;
				circle.m_radius = 2.0f;

				circle.m_p.Set(-10.0f, y + b + L);
				ground.CreateFixture(&circle, 0.0f);

				circle.m_p.Set(10.0f, y + b + L);
				ground.CreateFixture(&circle, 0.0f);
			}

			{

				b2PolygonShape shape;
				shape.SetAsBox(a, b);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;

				//bd.fixedRotation = true;
				bd.position.Set(-10.0f, y);
				b2Body* body1 = m_world.CreateBody(&bd);
				body1.CreateFixture(&shape, 5.0f);

				bd.position.Set(10.0f, y);
				b2Body* body2 = m_world.CreateBody(&bd);
				body2.CreateFixture(&shape, 5.0f);

				b2PulleyJointDef pulleyDef;
				b2Vec2 anchor1(-10.0f, y + b);
				b2Vec2 anchor2(10.0f, y + b);
				b2Vec2 groundAnchor1(-10.0f, y + b + L);
				b2Vec2 groundAnchor2(10.0f, y + b + L);
				pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5f);

				m_joint1 = (b2PulleyJoint*)m_world.CreateJoint(&pulleyDef);
			}
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 0:
				break;
			}
		}

		public void Step(Settings* settings)
		{
			Test::Step(settings);

			float ratio = m_joint1.GetRatio();
			float L = m_joint1.GetCurrentLengthA() + ratio * m_joint1.GetCurrentLengthB();
			m_debugDraw.DrawString(5, m_textLine, "L1 + %4.2f * L2 = %4.2f", (float) ratio, (float) L);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		public static Test Create()
		{
			return new Pulleys();
		}

		b2PulleyJoint* m_joint1;
	};
}
