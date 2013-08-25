using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;

namespace Testbed.Tests {
	/// This test shows how to use a motor joint. A motor joint
	/// can be used to animate a dynamic body. With finite motor forces
	/// the body can be blocked by collision with other bodies.
	class MotorJoint : Test
	{
		public MotorJoint()
		{
			b2Body ground = null;
			{
				b2BodyDef bd = new b2BodyDef();
				ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-20.0f, 0.0f), new b2Vec2(20.0f, 0.0f));

				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = shape;

				ground.CreateFixture(fd);
			}

			// Define motorized body
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(0.0f, 8.0f);
				b2Body body = m_world.CreateBody(bd);

				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(2.0f, 0.5f);

				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = shape;
				fd.friction = 0.6f;
				fd.density = 2.0f;
				body.CreateFixture(fd);

				b2MotorJointDef mjd;
				mjd.Initialize(ground, body);
				mjd.maxForce = 1000.0f;
				mjd.maxTorque = 1000.0f;
				m_joint = (b2MotorJoint*)m_world.CreateJoint(mjd);
			}

			m_go = false;
			m_time = 0.0f;
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 's':
				m_go = !m_go;
				break;
			}
		}

		public override void Step(Settings settings)
		{
			if (m_go && settings.hz > 0.0f)
			{
				m_time += 1.0f / settings.hz;
			}

			b2Vec2 linearOffset;
			linearOffset.x = 6.0f * (float)Math.Sin(2.0f * m_time);
			linearOffset.y = 8.0f + 4.0f * (float)Math.Sin(1.0f * m_time);
		
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
			return new MotorJoint();
		}

		b2MotorJoint m_joint;
		float m_time;
		bool m_go;
	};
}
