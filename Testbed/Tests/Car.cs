using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	// This is a fun demo that shows off the wheel joint
	class Car : Test
	{
		public Car()
		{		
			m_hz = 4.0f;
			m_zeta = 0.7f;
			m_speed = 50.0f;

			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 0.0f;
				fd.friction = 0.6f;

				shape.Set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
				ground.CreateFixture(fd);

				float[] hs = new float[]{0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

				float x = 20.0f, y1 = 0.0f, dx = 5.0f;

				for (int i = 0; i < 10; ++i)
				{
					float y2 = hs[i];
					shape.Set(new Vec2(x, y1), new Vec2(x + dx, y2));
					ground.CreateFixture(fd);
					y1 = y2;
					x += dx;
				}

				for (int i = 0; i < 10; ++i)
				{
					float y2 = hs[i];
					shape.Set(new Vec2(x, y1), new Vec2(x + dx, y2));
					ground.CreateFixture(fd);
					y1 = y2;
					x += dx;
				}

				shape.Set(new Vec2(x, 0.0f), new Vec2(x + 40.0f, 0.0f));
				ground.CreateFixture(fd);

				x += 80.0f;
				shape.Set(new Vec2(x, 0.0f), new Vec2(x + 40.0f, 0.0f));
				ground.CreateFixture(fd);

				x += 40.0f;
				shape.Set(new Vec2(x, 0.0f), new Vec2(x + 10.0f, 5.0f));
				ground.CreateFixture(fd);

				x += 20.0f;
				shape.Set(new Vec2(x, 0.0f), new Vec2(x + 40.0f, 0.0f));
				ground.CreateFixture(fd);

				x += 40.0f;
				shape.Set(new Vec2(x, 0.0f), new Vec2(x, 20.0f));
				ground.CreateFixture(fd);
			}

			// Teeter
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(140.0f, 1.0f);
				bd.type = BodyType._dynamicBody;
				Body body = m_world.CreateBody(bd);

				PolygonShape box = new PolygonShape();
				box.SetAsBox(10.0f, 0.25f);
				box.Density = 1;
				body.CreateFixture(box);

				RevoluteJointDef jd = new RevoluteJointDef();
				jd.Initialize(ground, body, body.GetPosition());
				jd.lowerAngle = -8.0f * (float)Math.PI / 180.0f;
				jd.upperAngle = 8.0f * (float)Math.PI / 180.0f;
				jd.enableLimit = true;
				m_world.CreateJoint(jd);

				body.ApplyAngularImpulse(100.0f, true);
			}

			// Bridge
			{
				int N = 20;
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(1.0f, 0.125f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 1.0f;
				fd.friction = 0.6f;

				RevoluteJointDef jd = new RevoluteJointDef();

				Body prevBody = ground;
				for (int i = 0; i < N; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(161.0f + 2.0f * i, -0.125f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					Vec2 anchor = new Vec2(160.0f + 2.0f * i, -0.125f);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(jd);

					prevBody = body;
				}

				Vec2 anchor2 = new Vec2(160.0f + 2.0f * N, -0.125f);
				jd.Initialize(prevBody, ground, anchor2);
				m_world.CreateJoint(jd);
			}

			// Boxes
			{
				PolygonShape box = new PolygonShape();
				box.SetAsBox(0.5f, 0.5f);
				box.Density = 0.5f;

				Body body = null;
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;

				bd.Position.Set(230.0f, 0.5f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(box);

				bd.Position.Set(230.0f, 1.5f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(box);

				bd.Position.Set(230.0f, 2.5f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(box);

				bd.Position.Set(230.0f, 3.5f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(box);

				bd.Position.Set(230.0f, 4.5f);
				body = m_world.CreateBody(bd);
				body.CreateFixture(box);
			}

			// Car
			{
				PolygonShape chassis = new PolygonShape();
				Vec2[] vertices = new Vec2[8];
				vertices[0].Set(-1.5f, -0.5f);
				vertices[1].Set(1.5f, -0.5f);
				vertices[2].Set(1.5f, 0.0f);
				vertices[3].Set(0.0f, 0.9f);
				vertices[4].Set(-1.15f, 0.9f);
				vertices[5].Set(-1.5f, 0.2f);
				chassis.Set(vertices, 6);

				CircleShape circle = new CircleShape();
				circle.m_radius = 0.4f;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(0.0f, 1.0f);
				m_car = m_world.CreateBody(bd);
				m_car.CreateFixture(chassis);

				FixtureDef fd = new FixtureDef();
				fd.shape = circle;
				fd.Density = 1.0f;
				fd.friction = 0.9f;

				bd.Position.Set(-1.0f, 0.35f);
				m_wheel1 = m_world.CreateBody(bd);
				m_wheel1.CreateFixture(fd);

				bd.Position.Set(1.0f, 0.4f);
				m_wheel2 = m_world.CreateBody(bd);
				m_wheel2.CreateFixture(fd);

				WheelJointDef jd = new WheelJointDef();
				Vec2 axis = new Vec2(0.0f, 1.0f);

				jd.Initialize(m_car, m_wheel1, m_wheel1.GetPosition(), axis);
				jd.motorSpeed = 0.0f;
				jd.maxMotorTorque = 20.0f;
				jd.enableMotor = true;
				jd.frequencyHz = m_hz;
				jd.dampingRatio = m_zeta;
				m_spring1 = (WheelJoint)m_world.CreateJoint(jd);

				jd.Initialize(m_car, m_wheel2, m_wheel2.GetPosition(), axis);
				jd.motorSpeed = 0.0f;
				jd.maxMotorTorque = 10.0f;
				jd.enableMotor = false;
				jd.frequencyHz = m_hz;
				jd.dampingRatio = m_zeta;
				m_spring2 = (WheelJoint)m_world.CreateJoint(jd);
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.A)){
				m_spring1.SetMotorSpeed(m_speed);
			}

			if (KeyboardManager.IsPressed(Key.S)){
				m_spring1.SetMotorSpeed(0.0f);
			}

			if (KeyboardManager.IsPressed(Key.D)){
				m_spring1.SetMotorSpeed(-m_speed);
			}

			if (KeyboardManager.IsPressed(Key.Q)){
				m_hz = Math.Max(0.0f, m_hz - 1.0f);
				m_spring1.SetSpringFrequencyHz(m_hz);
				m_spring2.SetSpringFrequencyHz(m_hz);
			}

			if (KeyboardManager.IsPressed(Key.E)) {
				m_hz += 1.0f;
				m_spring1.SetSpringFrequencyHz(m_hz);
				m_spring2.SetSpringFrequencyHz(m_hz);
			}
		}

		public override void Step(TestSettings settings)
		{
			m_debugDraw.DrawString("Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
			
			m_debugDraw.DrawString("frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
			

			settings.viewCenter.X = m_car.GetPosition().X;
			base.Step(settings);
		}

		public static Test Create()
		{
			return new Car();
		}

		Body m_car;
		Body m_wheel1;
		Body m_wheel2;

		float m_hz;
		float m_zeta;
		float m_speed;
		WheelJoint m_spring1;
		WheelJoint m_spring2;
	};
}
