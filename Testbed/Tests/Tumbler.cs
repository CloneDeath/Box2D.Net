using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Tumbler : Test
	{
		public enum
		{
			e_count = 800
		};

		public Tumbler()
		{
			b2Body* ground = null;
			{
				b2BodyDef bd;
				ground = m_world.CreateBody(&bd);
			}

			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.allowSleep = false;
				bd.position.Set(0.0f, 10.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2PolygonShape shape;
				shape.SetAsBox(0.5f, 10.0f, b2Vec2( 10.0f, 0.0f), 0.0);
				body.CreateFixture(&shape, 5.0f);
				shape.SetAsBox(0.5f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0);
				body.CreateFixture(&shape, 5.0f);
				shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, 10.0f), 0.0);
				body.CreateFixture(&shape, 5.0f);
				shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, -10.0f), 0.0);
				body.CreateFixture(&shape, 5.0f);

				b2RevoluteJointDef jd;
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.localAnchorA.Set(0.0f, 10.0f);
				jd.localAnchorB.Set(0.0f, 0.0f);
				jd.referenceAngle = 0.0f;
				jd.motorSpeed = 0.05f * Math.PI;
				jd.maxMotorTorque = 1e8f;
				jd.enableMotor = true;
				m_joint = (b2RevoluteJoint*)m_world.CreateJoint(&jd);
			}

			m_count = 0;
		}

		public void Step(Settings* settings)
		{
			Test::Step(settings);

			if (m_count < e_count)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 10.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2PolygonShape shape;
				shape.SetAsBox(0.125f, 0.125f);
				body.CreateFixture(&shape, 1.0f);

				++m_count;
			}
		}

		public static Test Create()
		{
			return new Tumbler();
		}

		b2RevoluteJoint* m_joint;
		int m_count;
	};
}
