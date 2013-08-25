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

			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape edge = new EdgeShape();
				edge.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				//ground.CreateFixture(shape, 0.0f);

				CircleShape circle = new CircleShape();
				circle.m_radius = 2.0f;

				circle.m_p.Set(-10.0f, y + b + L);
				circle.Density = 0;
				ground.CreateFixture(circle);

				circle.m_p.Set(10.0f, y + b + L);
				ground.CreateFixture(circle);
			}

			{

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(a, b);
				shape.Density = 5;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;

				//bd.fixedRotation = true;
				bd.Position.Set(-10.0f, y);
				Body body1 = m_world.CreateBody(bd);
				body1.CreateFixture(shape);

				bd.Position.Set(10.0f, y);
				Body body2 = m_world.CreateBody(bd);
				body2.CreateFixture(shape);

				PulleyJointDef pulleyDef = new PulleyJointDef();
				Vec2 anchor1 = new Vec2(-10.0f, y + b);
				Vec2 anchor2 = new Vec2(10.0f, y + b);
				Vec2 groundAnchor1 = new Vec2(-10.0f, y + b + L);
				Vec2 groundAnchor2 = new Vec2(10.0f, y + b + L);
				pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5f);

				m_joint1 = (PulleyJoint)m_world.CreateJoint(pulleyDef);
			}
		}

		public override void Keyboard()
		{
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			float ratio = m_joint1.GetRatio();
			float L = m_joint1.GetCurrentLengthA() + ratio * m_joint1.GetCurrentLengthB();
			m_debugDraw.DrawString("L1 + %4.2f * L2 = %4.2f", (float) ratio, (float) L);
			
		}

		public static Test Create()
		{
			return new Pulleys();
		}

		PulleyJoint m_joint1;
	};
}
