using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	// The motor in this test gets smoother with higher velocity iterations.
	class Prismatic : Test {
		public Prismatic() {
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(2.0f, 0.5f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-10.0f, 10.0f);
				bd.angle = 0.5f * (float)Math.PI;
				bd.allowSleep = false;
				Body body = m_world.CreateBody(bd);
				body.CreateFixture(shape, 5.0f);

				PrismaticJointDef pjd = new PrismaticJointDef();

				// Bouncy limit
				Vec2 axis = new Vec2(2.0f, 1.0f);
				axis.Normalize();
				pjd.Initialize(ground, body, new Vec2(0.0f, 0.0f), axis);

				// Non-bouncy limit
				//pjd.Initialize(ground, body, new Vec2(-10.0f, 10.0f), new Vec2(1.0f, 0.0f));

				pjd.motorSpeed = 10.0f;
				pjd.maxMotorForce = 10000.0f;
				pjd.enableMotor = true;
				pjd.lowerTranslation = 0.0f;
				pjd.upperTranslation = 20.0f;
				pjd.enableLimit = true;

				m_joint = (PrismaticJoint)m_world.CreateJoint(pjd);
			}
		}

		public override void Keyboard() {
			if (KeyboardManager.IsPressed(Key.L)) {
				m_joint.EnableLimit(!m_joint.IsLimitEnabled());
			}
			if (KeyboardManager.IsPressed(Key.M)) {
				m_joint.EnableMotor(!m_joint.IsMotorEnabled());
			}

			if (KeyboardManager.IsPressed(Key.S)) {
				m_joint.SetMotorSpeed(-m_joint.GetMotorSpeed());
			}
		}

		public override void Step(TestSettings settings) {
			base.Step(settings);
			m_debugDraw.DrawString("Keys: (l) limits, (m) motors, (s) speed");

			float force = m_joint.GetMotorForce(settings.hz);
			m_debugDraw.DrawString("Motor Force = %4.0f", (float)force);

		}

		public static Test Create() {
			return new Prismatic();
		}

		PrismaticJoint m_joint;
	};
}
