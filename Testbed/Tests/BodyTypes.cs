﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class BodyTypes : Test
	{
	public BodyTypes()
		{
			b2Body ground = null;
			{
				b2BodyDef bd = new b2BodyDef();
				ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-20.0f, 0.0f), new b2Vec2(20.0f, 0.0f));

				b2FixtureDef fd;
				fd.shape = shape;

				ground.CreateFixture(&fd);
			}

			// Define attachment
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(0.0f, 3.0f);
				m_attachment = m_world.CreateBody(bd);

				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.5f, 2.0f);
				m_attachment.CreateFixture(shape, 2.0f);
			}

			// Define platform
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(-4.0f, 5.0f);
				m_platform = m_world.CreateBody(bd);

				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.5f, 4.0f, new b2Vec2(4.0f, 0.0f), 0.5f * (float)Math.PI);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.friction = 0.6f;
				fd.density = 2.0f;
				m_platform.CreateFixture(&fd);

				b2RevoluteJointDef rjd;
				rjd.Initialize(m_attachment, m_platform, new b2Vec2(0.0f, 5.0f));
				rjd.maxMotorTorque = 50.0f;
				rjd.enableMotor = true;
				m_world.CreateJoint(&rjd);

				b2PrismaticJointDef pjd;
				pjd.Initialize(ground, m_platform, new b2Vec2(0.0f, 5.0f), new b2Vec2(1.0f, 0.0f));

				pjd.maxMotorForce = 1000.0f;
				pjd.enableMotor = true;
				pjd.lowerTranslation = -10.0f;
				pjd.upperTranslation = 10.0f;
				pjd.enableLimit = true;

				m_world.CreateJoint(&pjd);

				m_speed = 3.0f;
			}

			// Create a payload
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(0.0f, 8.0f);
				b2Body body = m_world.CreateBody(bd);

				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.75f, 0.75f);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.friction = 0.6f;
				fd.density = 2.0f;

				body.CreateFixture(fd);
			}
		}

		public void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.D)){
				m_platform.SetType(b2BodyType.b2_dynamicBody);
			}

			if (KeyboardManager.IsPressed(Key.S)){
				m_platform.SetType(b2BodyType.b2_staticBody);
			}

			if (KeyboardManager.IsPressed(Key.K)){
				m_platform.SetType(b2BodyType.b2_kinematicBody);
				m_platform.SetLinearVelocity(new b2Vec2(-m_speed, 0.0f));
				m_platform.SetAngularVelocity(0.0f);
			}
		}

		public override void Step(Settings settings)
		{
			// Drive the kinematic body.
			if (m_platform.GetBodyType() == b2BodyType.b2_kinematicBody)
			{
				b2Vec2 p = m_platform.GetTransform().p;
				b2Vec2 v = m_platform.GetLinearVelocity();

				if ((p.x < -10.0f && v.x < 0.0f) ||
					(p.x > 10.0f && v.x > 0.0f))
				{
					v.x = -v.x;
					m_platform.SetLinearVelocity(v);
				}
			}

			base.Step(settings);
			m_debugDraw.DrawString("Keys: (d) dynamic, (s) static, (k) kinematic");
		}

		public static Test Create()
		{
			return new BodyTypes();
		}

		b2Body m_attachment;
		b2Body m_platform;
		float m_speed;
	};
}
