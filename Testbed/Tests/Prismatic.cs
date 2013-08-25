using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	// The motor in this test gets smoother with higher velocity iterations.
	class Prismatic : Test
	{
		public Prismatic()
		{
			b2Body* ground = null;
			{
				b2BodyDef bd;
				ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				shape.SetAsBox(2.0f, 0.5f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-10.0f, 10.0f);
				bd.angle = 0.5f * Math.PI;
				bd.allowSleep = false;
				b2Body* body = m_world.CreateBody(&bd);
				body.CreateFixture(&shape, 5.0f);

				b2PrismaticJointDef pjd;

				// Bouncy limit
				b2Vec2 axis(2.0f, 1.0f);
				axis.Normalize();
				pjd.Initialize(ground, body, b2Vec2(0.0f, 0.0f), axis);

				// Non-bouncy limit
				//pjd.Initialize(ground, body, b2Vec2(-10.0f, 10.0f), b2Vec2(1.0f, 0.0f));

				pjd.motorSpeed = 10.0f;
				pjd.maxMotorForce = 10000.0f;
				pjd.enableMotor = true;
				pjd.lowerTranslation = 0.0f;
				pjd.upperTranslation = 20.0f;
				pjd.enableLimit = true;

				m_joint = (b2PrismaticJoint*)m_world.CreateJoint(&pjd);
			}
		}

		public void Keyboard(unsigned char key)
		{
			switch (key)
			{
			case 'l':
				m_joint.EnableLimit(!m_joint.IsLimitEnabled());
				break;

			case 'm':
				m_joint.EnableMotor(!m_joint.IsMotorEnabled());
				break;

			case 's':
				m_joint.SetMotorSpeed(-m_joint.GetMotorSpeed());
				break;
			}
		}

		public void Step(Settings* settings)
		{
			Test::Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Keys: (l) limits, (m) motors, (s) speed");
			m_textLine += DRAW_STRING_NEW_LINE;
			float force = m_joint.GetMotorForce(settings.hz);
			m_debugDraw.DrawString(5, m_textLine, "Motor Force = %4.0f", (float) force);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		public static Test Create()
		{
			return new Prismatic();
		}

		b2PrismaticJoint* m_joint;
	};
}
