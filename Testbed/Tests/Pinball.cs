﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	
	/// This tests bullet collision and provides an example of a gameplay scenario.
	/// This also uses a loop shape.
	class Pinball : Test
	{
	public:
		Pinball()
		{
			// Ground body
			b2Body* ground = null;
			{
				b2BodyDef bd;
				ground = m_world.CreateBody(&bd);

				b2Vec2 vs[5];
				vs[0].Set(0.0f, -2.0f);
				vs[1].Set(8.0f, 6.0f);
				vs[2].Set(8.0f, 20.0f);
				vs[3].Set(-8.0f, 20.0f);
				vs[4].Set(-8.0f, 6.0f);

				b2ChainShape loop;
				loop.CreateLoop(vs, 5);
				b2FixtureDef fd;
				fd.shape = &loop;
				fd.density = 0.0f;
				ground.CreateFixture(&fd);
			}

			// Flippers
			{
				b2Vec2 p1(-2.0f, 0.0f), p2(2.0f, 0.0f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;

				bd.position = p1;
				b2Body* leftFlipper = m_world.CreateBody(&bd);

				bd.position = p2;
				b2Body* rightFlipper = m_world.CreateBody(&bd);

				b2PolygonShape box;
				box.SetAsBox(1.75f, 0.1f);

				b2FixtureDef fd;
				fd.shape = &box;
				fd.density = 1.0f;

				leftFlipper.CreateFixture(&fd);
				rightFlipper.CreateFixture(&fd);

				b2RevoluteJointDef jd;
				jd.bodyA = ground;
				jd.localAnchorB.SetZero();
				jd.enableMotor = true;
				jd.maxMotorTorque = 1000.0f;
				jd.enableLimit = true;

				jd.motorSpeed = 0.0f;
				jd.localAnchorA = p1;
				jd.bodyB = leftFlipper;
				jd.lowerAngle = -30.0f * Math.PI / 180.0f;
				jd.upperAngle = 5.0f * Math.PI / 180.0f;
				m_leftJoint = (b2RevoluteJoint*)m_world.CreateJoint(&jd);

				jd.motorSpeed = 0.0f;
				jd.localAnchorA = p2;
				jd.bodyB = rightFlipper;
				jd.lowerAngle = -5.0f * Math.PI / 180.0f;
				jd.upperAngle = 30.0f * Math.PI / 180.0f;
				m_rightJoint = (b2RevoluteJoint*)m_world.CreateJoint(&jd);
			}

			// Circle character
			{
				b2BodyDef bd;
				bd.position.Set(1.0f, 15.0f);
				bd.type = b2_dynamicBody;
				bd.bullet = true;

				m_ball = m_world.CreateBody(&bd);

				b2CircleShape shape;
				shape.m_radius = 0.2f;

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 1.0f;
				m_ball.CreateFixture(&fd);
			}

			m_button = false;
		}

		void Step(Settings* settings)
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

			Test::Step(settings);

			m_debugDraw.DrawString(5, m_textLine, "Press 'a' to control the flippers");
			m_textLine += DRAW_STRING_NEW_LINE;

		}

		void Keyboard(unsigned char key)
		{
			switch (key)
			{
			case 'a':
			case 'A':
				m_button = true;
				break;
			}
		}

		void KeyboardUp(unsigned char key)
		{
			switch (key)
			{
			case 'a':
			case 'A':
				m_button = false;
				break;
			}
		}

		static Test* Create()
		{
			return new Pinball;
		}

		b2RevoluteJoint* m_leftJoint;
		b2RevoluteJoint* m_rightJoint;
		b2Body* m_ball;
		bool m_button;
	};
}