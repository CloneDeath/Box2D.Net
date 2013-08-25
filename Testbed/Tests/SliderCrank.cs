using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// A motor driven slider crank with joint friction.

	class SliderCrank : Test
	{
		public SliderCrank()
		{
			b2Body ground = null;
			{
				b2BodyDef bd = new b2BodyDef();
				ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				b2Body prevBody = ground;

				// Define crank.
				{
					b2PolygonShape shape = new b2PolygonShape();
					shape.SetAsBox(0.5f, 2.0f);

					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(0.0f, 7.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);

					b2RevoluteJointDef rjd = new b2RevoluteJointDef();
					rjd.Initialize(prevBody, body, new b2Vec2(0.0f, 5.0f));
					rjd.motorSpeed = 1.0f * (float)Math.PI;
					rjd.maxMotorTorque = 10000.0f;
					rjd.enableMotor = true;
					m_joint1 = (b2RevoluteJoint)m_world.CreateJoint(rjd);

					prevBody = body;
				}

				// Define follower.
				{
					b2PolygonShape shape = new b2PolygonShape();
					shape.SetAsBox(0.5f, 4.0f);

					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(0.0f, 13.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);

					b2RevoluteJointDef rjd = new b2RevoluteJointDef();
					rjd.Initialize(prevBody, body, new b2Vec2(0.0f, 9.0f));
					rjd.enableMotor = false;
					m_world.CreateJoint(rjd);

					prevBody = body;
				}

				// Define piston
				{
					b2PolygonShape shape = new b2PolygonShape();
					shape.SetAsBox(1.5f, 1.5f);

					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.fixedRotation = true;
					bd.position.Set(0.0f, 17.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);

					b2RevoluteJointDef rjd = new b2RevoluteJointDef();
					rjd.Initialize(prevBody, body, new b2Vec2(0.0f, 17.0f));
					m_world.CreateJoint(rjd);

					b2PrismaticJointDef pjd = new b2PrismaticJointDef();
					pjd.Initialize(ground, body, new b2Vec2(0.0f, 17.0f), new b2Vec2(0.0f, 1.0f));

					pjd.maxMotorForce = 1000.0f;
					pjd.enableMotor = true;

					m_joint2 = (b2PrismaticJoint)m_world.CreateJoint(pjd);
				}

				// Create a payload
				{
					b2PolygonShape shape = new b2PolygonShape();
					shape.SetAsBox(1.5f, 1.5f);

					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(0.0f, 23.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);
				}
			}
		}

		public override void Keyboard()
		{
			switch (key)
			{
			case 'f':
				m_joint2.EnableMotor(!m_joint2.IsMotorEnabled());
				m_joint2.GetBodyB().SetAwake(true);
				break;

			case 'm':
				m_joint1.EnableMotor(!m_joint1.IsMotorEnabled());
				m_joint1.GetBodyB().SetAwake(true);
				break;
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Keys: (f) toggle friction, (m) toggle motor");
			
			float torque = m_joint1.GetMotorTorque(settings.hz);
			m_debugDraw.DrawString("Motor Torque = %5.0f", (float) torque);
			
		}

		public static Test Create()
		{
			return new SliderCrank();
		}

		b2RevoluteJoint m_joint1;
		b2PrismaticJoint m_joint2;
	};
}
