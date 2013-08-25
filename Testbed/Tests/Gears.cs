using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Gears : Test
	{
		public Gears()
		{
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(50.0f, 0.0f), new Vec2(-50.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				CircleShape circle1 = new CircleShape();
				circle1.m_radius = 1.0f;

				PolygonShape box = new PolygonShape();
				box.SetAsBox(0.5f, 5.0f);

				CircleShape circle2 = new CircleShape();
				circle2.m_radius = 2.0f;
			
				BodyDef bd1 = new BodyDef();
				bd1.type = BodyType._staticBody;
				bd1.position.Set(10.0f, 9.0f);
				Body body1 = m_world.CreateBody(bd1);
				body1.CreateFixture(circle1, 5.0f);

				BodyDef bd2 = new BodyDef();
				bd2.type = BodyType._dynamicBody;
				bd2.position.Set(10.0f, 8.0f);
				Body body2 = m_world.CreateBody(bd2);
				body2.CreateFixture(box, 5.0f);

				BodyDef bd3 = new BodyDef();
				bd3.type = BodyType._dynamicBody;
				bd3.position.Set(10.0f, 6.0f);
				Body body3 = m_world.CreateBody(bd3);
				body3.CreateFixture(circle2, 5.0f);

				RevoluteJointDef jd1 = new RevoluteJointDef();
				jd1.Initialize(body2, body1, bd1.position);
				Joint joint1 = m_world.CreateJoint(jd1);

				RevoluteJointDef jd2 = new RevoluteJointDef();
				jd2.Initialize(body2, body3, bd3.position);
				Joint joint2 = m_world.CreateJoint(jd2);

				GearJointDef jd4 = new GearJointDef();
				jd4.bodyA = body1;
				jd4.bodyB = body3;
				jd4.joint1 = joint1;
				jd4.joint2 = joint2;
				jd4.ratio = circle2.m_radius / circle1.m_radius;
				m_world.CreateJoint(jd4);
			}

			{
				CircleShape circle1 = new CircleShape();
				circle1.m_radius = 1.0f;

				CircleShape circle2 = new CircleShape();
				circle2.m_radius = 2.0f;
			
				PolygonShape box = new PolygonShape();
				box.SetAsBox(0.5f, 5.0f);

				BodyDef bd1 = new BodyDef();
				bd1.type = BodyType._dynamicBody;
				bd1.position.Set(-3.0f, 12.0f);
				Body body1 = m_world.CreateBody(bd1);
				body1.CreateFixture(circle1, 5.0f);

				RevoluteJointDef jd1 = new RevoluteJointDef();
				jd1.bodyA = ground;
				jd1.bodyB = body1;
				jd1.localAnchorA = ground.GetLocalPoint(bd1.position);
				jd1.localAnchorB = body1.GetLocalPoint(bd1.position);
				jd1.referenceAngle = body1.GetAngle() - ground.GetAngle();
				m_joint1 = (RevoluteJoint)m_world.CreateJoint(jd1);

				BodyDef bd2 = new BodyDef();
				bd2.type = BodyType._dynamicBody;
				bd2.position.Set(0.0f, 12.0f);
				Body body2 = m_world.CreateBody(bd2);
				body2.CreateFixture(circle2, 5.0f);

				RevoluteJointDef jd2 = new RevoluteJointDef();
				jd2.Initialize(ground, body2, bd2.position);
				m_joint2 = (RevoluteJoint)m_world.CreateJoint(jd2);

				BodyDef bd3 = new BodyDef();
				bd3.type = BodyType._dynamicBody;
				bd3.position.Set(2.5f, 12.0f);
				Body body3 = m_world.CreateBody(bd3);
				body3.CreateFixture(box, 5.0f);

				PrismaticJointDef jd3 = new PrismaticJointDef();
				jd3.Initialize(ground, body3, bd3.position, new Vec2(0.0f, 1.0f));
				jd3.lowerTranslation = -5.0f;
				jd3.upperTranslation = 5.0f;
				jd3.enableLimit = true;

				m_joint3 = (PrismaticJoint)m_world.CreateJoint(jd3);

				GearJointDef jd4 = new GearJointDef();
				jd4.bodyA = body1;
				jd4.bodyB = body2;
				jd4.joint1 = m_joint1;
				jd4.joint2 = m_joint2;
				jd4.ratio = circle2.m_radius / circle1.m_radius;
				m_joint4 = (GearJoint)m_world.CreateJoint(jd4);

				GearJointDef jd5 = new GearJointDef();
				jd5.bodyA = body2;
				jd5.bodyB = body3;
				jd5.joint1 = m_joint2;
				jd5.joint2 = m_joint3;
				jd5.ratio = -1.0f / circle2.m_radius;
				m_joint5 = (GearJoint)m_world.CreateJoint(jd5);
			}
		}

		public override void Keyboard()
		{
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			float ratio, value;
		
			ratio = m_joint4.GetRatio();
			value = m_joint1.GetJointAngle() + ratio * m_joint2.GetJointAngle();
			m_debugDraw.DrawString("theta1 + %4.2f * theta2 = %4.2f", (float) ratio, (float) value);
			

			ratio = m_joint5.GetRatio();
			value = m_joint2.GetJointAngle() + ratio * m_joint3.GetJointTranslation();
			m_debugDraw.DrawString("theta2 + %4.2f * delta = %4.2f", (float) ratio, (float) value);
			
		}

		public static Test Create()
		{
			return new Gears();
		}

		RevoluteJoint m_joint1;
		RevoluteJoint m_joint2;
		PrismaticJoint m_joint3;
		GearJoint m_joint4;
		GearJoint m_joint5;
	};
}
