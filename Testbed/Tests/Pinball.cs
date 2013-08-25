using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	
	/// This tests bullet collision and provides an example of a gameplay scenario.
	/// This also uses a loop shape.
	class Pinball : Test
	{
		public Pinball()
		{
			// Ground body
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				Vec2[] vs = new Vec2[5];
				vs[0].Set(0.0f, -2.0f);
				vs[1].Set(8.0f, 6.0f);
				vs[2].Set(8.0f, 20.0f);
				vs[3].Set(-8.0f, 20.0f);
				vs[4].Set(-8.0f, 6.0f);

				ChainShape loop = new ChainShape();
				loop.CreateLoop(vs, 5);
				FixtureDef fd = new FixtureDef();
				fd.shape = loop;
				fd.density = 0.0f;
				ground.CreateFixture(fd);
			}

			// Flippers
			{
				Vec2 p1 = new Vec2(-2.0f, 0.0f);
				Vec2 p2 = new Vec2(2.0f, 0.0f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;

				bd.position = p1;
				Body leftFlipper = m_world.CreateBody(bd);

				bd.position = p2;
				Body rightFlipper = m_world.CreateBody(bd);

				PolygonShape box = new PolygonShape();
				box.SetAsBox(1.75f, 0.1f);

				FixtureDef fd = new FixtureDef();
				fd.shape = box;
				fd.density = 1.0f;

				leftFlipper.CreateFixture(fd);
				rightFlipper.CreateFixture(fd);

				RevoluteJointDef jd = new RevoluteJointDef();
				jd.bodyA = ground;
				jd.localAnchorB.SetZero();
				jd.enableMotor = true;
				jd.maxMotorTorque = 1000.0f;
				jd.enableLimit = true;

				jd.motorSpeed = 0.0f;
				jd.localAnchorA = p1;
				jd.bodyB = leftFlipper;
				jd.lowerAngle = -30.0f * (float)Math.PI / 180.0f;
				jd.upperAngle = 5.0f * (float)Math.PI / 180.0f;
				m_leftJoint = (RevoluteJoint)m_world.CreateJoint(jd);

				jd.motorSpeed = 0.0f;
				jd.localAnchorA = p2;
				jd.bodyB = rightFlipper;
				jd.lowerAngle = -5.0f * (float)Math.PI / 180.0f;
				jd.upperAngle = 30.0f * (float)Math.PI / 180.0f;
				m_rightJoint = (RevoluteJoint)m_world.CreateJoint(jd);
			}

			// Circle character
			{
				BodyDef bd = new BodyDef();
				bd.position.Set(1.0f, 15.0f);
				bd.type = BodyType._dynamicBody;
				bd.bullet = true;

				m_ball = m_world.CreateBody(bd);

				CircleShape shape = new CircleShape();
				shape.m_radius = 0.2f;

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.density = 1.0f;
				m_ball.CreateFixture(fd);
			}

			m_button = false;
		}

		public override void Step(TestSettings settings)
		{
			if (m_button)
			{
				m_leftJoint.SetMotorSpeed(20.0f);
				m_rightJoint.SetMotorSpeed(-20.0f);
			}
			else
			{
				m_leftJoint.SetMotorSpeed(-10.0f);
				m_rightJoint.SetMotorSpeed(10.0f);
			}

			base.Step(settings);

			m_debugDraw.DrawString("Press 'a' to control the flippers");
			

		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.A)) {
				m_button = true;
			} else if (KeyboardManager.IsReleased(Key.A)) {
				m_button = false;
			}
		}

		public static Test Create()
		{
			return new Pinball();
		}

		RevoluteJoint m_leftJoint;
		RevoluteJoint m_rightJoint;
		Body m_ball;
		bool m_button;
	};
}
