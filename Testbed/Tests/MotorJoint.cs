﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	/// This test shows how to use a motor joint. A motor joint
	/// can be used to animate a dynamic body. With finite motor forces
	/// the body can be blocked by collision with other bodies.
	class MotorJoint : Test
	{
	public:
		MotorJoint()
		{
			b2Body* ground = null;
			{
				b2BodyDef bd;
				ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));

				b2FixtureDef fd;
				fd.shape = &shape;

				ground.CreateFixture(&fd);
			}

			// Define motorized body
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 8.0f);
				b2Body* body = m_world.CreateBody(&bd);

				b2PolygonShape shape;
				shape.SetAsBox(2.0f, 0.5f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.friction = 0.6f;
				fd.density = 2.0f;
				body.CreateFixture(&fd);

				b2MotorJointDef mjd;
				mjd.Initialize(ground, body);
				mjd.maxForce = 1000.0f;
				mjd.maxTorque = 1000.0f;
				m_joint = (b2MotorJoint*)m_world.CreateJoint(&mjd);
			}

			m_go = false;
			m_time = 0.0f;
		}

		void Keyboard(unsigned char key)
		{
			switch (key)
			{
			case 's':
				m_go = !m_go;
				break;
			}
		}

		void Step(Settings* settings)
		{
			if (m_go && settings.hz > 0.0f)
			{
				m_time += 1.0f / settings.hz;
			}

			b2Vec2 linearOffset;
			linearOffset.x = 6.0f * sinf(2.0f * m_time);
			linearOffset.y = 8.0f + 4.0f * sinf(1.0f * m_time);
		
			float angularOffset = 4.0f * m_time;

			m_joint.SetLinearOffset(linearOffset);
			m_joint.SetAngularOffset(angularOffset);

			m_debugDraw.DrawPoint(linearOffset, 4.0f, b2Color(0.9f, 0.9f, 0.9f));

			Test::Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Keys: (s) pause");
			m_textLine += 15;
		}

		static Test* Create()
		{
			return new MotorJoint;
		}

		b2MotorJoint* m_joint;
		float m_time;
		bool m_go;
	};
}