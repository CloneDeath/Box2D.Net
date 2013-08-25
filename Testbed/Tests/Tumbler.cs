using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Tumbler : Test
	{
		const int e_count = 800;

		public Tumbler()
		{
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);
			}

			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.allowSleep = false;
				bd.position.Set(0.0f, 10.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 10.0f, new Vec2( 10.0f, 0.0f), 0.0f);
				body.CreateFixture(shape, 5.0f);
				shape.SetAsBox(0.5f, 10.0f, new Vec2(-10.0f, 0.0f), 0.0f);
				body.CreateFixture(shape, 5.0f);
				shape.SetAsBox(10.0f, 0.5f, new Vec2(0.0f, 10.0f), 0.0f);
				body.CreateFixture(shape, 5.0f);
				shape.SetAsBox(10.0f, 0.5f, new Vec2(0.0f, -10.0f), 0.0f);
				body.CreateFixture(shape, 5.0f);

				RevoluteJointDef jd = new RevoluteJointDef();
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.localAnchorA.Set(0.0f, 10.0f);
				jd.localAnchorB.Set(0.0f, 0.0f);
				jd.referenceAngle = 0.0f;
				jd.motorSpeed = 0.05f * (float)Math.PI;
				jd.maxMotorTorque = 1e8f;
				jd.enableMotor = true;
				m_joint = (RevoluteJoint)m_world.CreateJoint(jd);
			}

			m_count = 0;
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			if (m_count < e_count)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(0.0f, 10.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.125f, 0.125f);
				body.CreateFixture(shape, 1.0f);

				++m_count;
			}
		}

		public static Test Create()
		{
			return new Tumbler();
		}

		RevoluteJoint m_joint;
		int m_count;
	};
}
