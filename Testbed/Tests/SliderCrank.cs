using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	// A motor driven slider crank with joint friction.

	class SliderCrank : Test
	{
		public SliderCrank()
		{
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				Body prevBody = ground;

				// Define crank.
				{
					PolygonShape shape = new PolygonShape();
					shape.SetAsBox(0.5f, 2.0f);

					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.position.Set(0.0f, 7.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);

					RevoluteJointDef rjd = new RevoluteJointDef();
					rjd.Initialize(prevBody, body, new Vec2(0.0f, 5.0f));
					rjd.motorSpeed = 1.0f * (float)Math.PI;
					rjd.maxMotorTorque = 10000.0f;
					rjd.enableMotor = true;
					m_joint1 = (RevoluteJoint)m_world.CreateJoint(rjd);

					prevBody = body;
				}

				// Define follower.
				{
					PolygonShape shape = new PolygonShape();
					shape.SetAsBox(0.5f, 4.0f);

					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.position.Set(0.0f, 13.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);

					RevoluteJointDef rjd = new RevoluteJointDef();
					rjd.Initialize(prevBody, body, new Vec2(0.0f, 9.0f));
					rjd.enableMotor = false;
					m_world.CreateJoint(rjd);

					prevBody = body;
				}

				// Define piston
				{
					PolygonShape shape = new PolygonShape();
					shape.SetAsBox(1.5f, 1.5f);

					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.fixedRotation = true;
					bd.position.Set(0.0f, 17.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);

					RevoluteJointDef rjd = new RevoluteJointDef();
					rjd.Initialize(prevBody, body, new Vec2(0.0f, 17.0f));
					m_world.CreateJoint(rjd);

					PrismaticJointDef pjd = new PrismaticJointDef();
					pjd.Initialize(ground, body, new Vec2(0.0f, 17.0f), new Vec2(0.0f, 1.0f));

					pjd.maxMotorForce = 1000.0f;
					pjd.enableMotor = true;

					m_joint2 = (PrismaticJoint)m_world.CreateJoint(pjd);
				}

				// Create a payload
				{
					PolygonShape shape = new PolygonShape();
					shape.SetAsBox(1.5f, 1.5f);

					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.position.Set(0.0f, 23.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 2.0f);
				}
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.F)){
				m_joint2.EnableMotor(!m_joint2.IsMotorEnabled());
				m_joint2.GetBodyB().SetAwake(true);
			}
			if (KeyboardManager.IsPressed(Key.M)) {
				m_joint1.EnableMotor(!m_joint1.IsMotorEnabled());
				m_joint1.GetBodyB().SetAwake(true);
			}
		}

		public override void Step(TestSettings settings)
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

		RevoluteJoint m_joint1;
		PrismaticJoint m_joint2;
	};
}
