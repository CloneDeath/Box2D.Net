using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	/// This test shows how to use a motor joint. A motor joint
	/// can be used to animate a dynamic body. With finite motor forces
	/// the body can be blocked by collision with other bodies.
	class MotorJointTest : Test
	{
		public MotorJointTest()
		{
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;

				ground.CreateFixture(fd);
			}

			// Define motorized body
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(0.0f, 8.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(2.0f, 0.5f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.friction = 0.6f;
				fd.Density = 2.0f;
				body.CreateFixture(fd);

				MotorJointDef mjd = new MotorJointDef();
				mjd.Initialize(ground, body);
				mjd.maxForce = 1000.0f;
				mjd.maxTorque = 1000.0f;
				m_joint = (MotorJoint)m_world.CreateJoint(mjd);
			}

			m_go = false;
			m_time = 0.0f;
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.S)) {
				m_go = !m_go;
			}
		}

		public override void Step(TestSettings settings)
		{
			if (m_go && settings.hz > 0.0f)
			{
				m_time += 1.0f / settings.hz;
			}

			Vec2 linearOffset;
			linearOffset.X = 6.0f * (float)Math.Sin(2.0f * m_time);
			linearOffset.Y = 8.0f + 4.0f * (float)Math.Sin(1.0f * m_time);
		
			float angularOffset = 4.0f * m_time;

			m_joint.SetLinearOffset(linearOffset);
			m_joint.SetAngularOffset(angularOffset);

			m_debugDraw.DrawPoint(linearOffset, 4.0f, Color.FromArgb(225, 225, 225));

			base.Step(settings);
			m_debugDraw.DrawString("Keys: (s) pause");
			m_textLine += 15;
		}

		public static Test Create()
		{
			return new MotorJointTest();
		}

		MotorJoint m_joint;
		float m_time;
		bool m_go;
	};
}
