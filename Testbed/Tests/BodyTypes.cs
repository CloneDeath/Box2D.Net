using System;
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

			// Define attachment
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(0.0f, 3.0f);
				m_attachment = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 2.0f);
				m_attachment.CreateFixture(shape, 2.0f);
			}

			// Define platform
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-4.0f, 5.0f);
				m_platform = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 4.0f, new Vec2(4.0f, 0.0f), 0.5f * (float)Math.PI);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.friction = 0.6f;
				fd.density = 2.0f;
				m_platform.CreateFixture(fd);

				RevoluteJointDef rjd = new RevoluteJointDef();
				rjd.Initialize(m_attachment, m_platform, new Vec2(0.0f, 5.0f));
				rjd.maxMotorTorque = 50.0f;
				rjd.enableMotor = true;
				m_world.CreateJoint(rjd);

				PrismaticJointDef pjd = new PrismaticJointDef();
				pjd.Initialize(ground, m_platform, new Vec2(0.0f, 5.0f), new Vec2(1.0f, 0.0f));

				pjd.maxMotorForce = 1000.0f;
				pjd.enableMotor = true;
				pjd.lowerTranslation = -10.0f;
				pjd.upperTranslation = 10.0f;
				pjd.enableLimit = true;

				m_world.CreateJoint(pjd);

				m_speed = 3.0f;
			}

			// Create a payload
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(0.0f, 8.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.75f, 0.75f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.friction = 0.6f;
				fd.density = 2.0f;

				body.CreateFixture(fd);
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.D)){
				m_platform.SetType(BodyType._dynamicBody);
			}

			if (KeyboardManager.IsPressed(Key.S)){
				m_platform.SetType(BodyType._staticBody);
			}

			if (KeyboardManager.IsPressed(Key.K)){
				m_platform.SetType(BodyType._kinematicBody);
				m_platform.SetLinearVelocity(new Vec2(-m_speed, 0.0f));
				m_platform.SetAngularVelocity(0.0f);
			}
		}

		public override void Step(TestSettings settings)
		{
			// Drive the kinematic body.
			if (m_platform.GetBodyType() == BodyType._kinematicBody)
			{
				Vec2 p = m_platform.GetTransform().p;
				Vec2 v = m_platform.GetLinearVelocity();

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

		Body m_attachment;
		Body m_platform;
		float m_speed;
	};
}
