using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	// Inspired by a contribution by roman_m
	// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)
	class TheoJansen : Test
	{
		public void CreateLeg(float s, Vec2 wheelAnchor)
		{
			Vec2 p1 = new Vec2(5.4f * s, -6.1f);
			Vec2 p2 = new Vec2(7.2f * s, -1.2f);
			Vec2 p3 = new Vec2(4.3f * s, -1.9f);
			Vec2 p4 = new Vec2(3.1f * s, 0.8f);
			Vec2 p5 = new Vec2(6.0f * s, 1.5f);
			Vec2 p6 = new Vec2(2.5f * s, 3.7f);

			FixtureDef fd1 = new FixtureDef();
			FixtureDef fd2 = new FixtureDef();
			fd1.filter.groupIndex = -1;
			fd2.filter.groupIndex = -1;
			fd1.density = 1.0f;
			fd2.density = 1.0f;

			PolygonShape poly1 = new PolygonShape();
			PolygonShape poly2 = new PolygonShape();

			if (s > 0.0f)
			{
				Vec2[] vertices = new Vec2[3];

				vertices[0] = p1;
				vertices[1] = p2;
				vertices[2] = p3;
				poly1.Set(vertices, 3);

				vertices[0] = new Vec2(0, 0);
				vertices[1] = p5 - p4;
				vertices[2] = p6 - p4;
				poly2.Set(vertices, 3);
			}
			else
			{
				Vec2[] vertices = new Vec2[3];

				vertices[0] = p1;
				vertices[1] = p3;
				vertices[2] = p2;
				poly1.Set(vertices, 3);

				vertices[0] = new Vec2(0, 0);
				vertices[1] = p6 - p4;
				vertices[2] = p5 - p4;
				poly2.Set(vertices, 3);
			}

			fd1.shape = poly1;
			fd2.shape = poly2;

			BodyDef bd1 = new BodyDef();
			BodyDef bd2 = new BodyDef();
			bd1.type = BodyType._dynamicBody;
			bd2.type = BodyType._dynamicBody;
			bd1.position = m_offset;
			bd2.position = p4 + m_offset;

			bd1.angularDamping = 10.0f;
			bd2.angularDamping = 10.0f;

			Body body1 = m_world.CreateBody(bd1);
			Body body2 = m_world.CreateBody(bd2);

			body1.CreateFixture(fd1);
			body2.CreateFixture(fd2);

			DistanceJointDef djd = new DistanceJointDef();

			// Using a soft distance constraint can reduce some jitter.
			// It also makes the structure seem a bit more fluid by
			// acting like a suspension system.
			djd.dampingRatio = 0.5f;
			djd.frequencyHz = 10.0f;

			djd.Initialize(body1, body2, p2 + m_offset, p5 + m_offset);
			m_world.CreateJoint(djd);

			djd.Initialize(body1, body2, p3 + m_offset, p4 + m_offset);
			m_world.CreateJoint(djd);

			djd.Initialize(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset);
			m_world.CreateJoint(djd);

			djd.Initialize(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset);
			m_world.CreateJoint(djd);

			RevoluteJointDef rjd = new RevoluteJointDef();

			rjd.Initialize(body2, m_chassis, p4 + m_offset);
			m_world.CreateJoint(rjd);
		}

		public TheoJansen()
		{
			m_offset.Set(0.0f, 8.0f);
			m_motorSpeed = 2.0f;
			m_motorOn = true;
			Vec2 pivot = new Vec2(0.0f, 0.8f);

			// Ground
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-50.0f, 0.0f), new Vec2(50.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);

				shape.Set(new Vec2(-50.0f, 0.0f), new Vec2(-50.0f, 10.0f));
				ground.CreateFixture(shape, 0.0f);

				shape.Set(new Vec2(50.0f, 0.0f), new Vec2(50.0f, 10.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			// Balls
			for (int i = 0; i < 40; ++i)
			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 0.25f;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-40.0f + 2.0f * i, 0.5f);

				Body body = m_world.CreateBody(bd);
				body.CreateFixture(shape, 1.0f);
			}

			// Chassis
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(2.5f, 1.0f);

				FixtureDef sd = new FixtureDef();
				sd.density = 1.0f;
				sd.shape = shape;
				sd.filter.groupIndex = -1;
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position = pivot + m_offset;
				m_chassis = m_world.CreateBody(bd);
				m_chassis.CreateFixture(sd);
			}

			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 1.6f;

				FixtureDef sd = new FixtureDef();
				sd.density = 1.0f;
				sd.shape = shape;
				sd.filter.groupIndex = -1;
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position = pivot + m_offset;
				m_wheel = m_world.CreateBody(bd);
				m_wheel.CreateFixture(sd);
			}

			{
				RevoluteJointDef jd = new RevoluteJointDef();
				jd.Initialize(m_wheel, m_chassis, pivot + m_offset);
				jd.collideConnected = false;
				jd.motorSpeed = m_motorSpeed;
				jd.maxMotorTorque = 400.0f;
				jd.enableMotor = m_motorOn;
				m_motorJoint = (RevoluteJoint)m_world.CreateJoint(jd);
			}

			Vec2 wheelAnchor;
		
			wheelAnchor = pivot + new Vec2(0.0f, -0.8f);

			CreateLeg(-1.0f, wheelAnchor);
			CreateLeg(1.0f, wheelAnchor);

			m_wheel.SetTransform(m_wheel.GetPosition(), 120.0f * (float)Math.PI / 180.0f);
			CreateLeg(-1.0f, wheelAnchor);
			CreateLeg(1.0f, wheelAnchor);

			m_wheel.SetTransform(m_wheel.GetPosition(), -120.0f * (float)Math.PI / 180.0f);
			CreateLeg(-1.0f, wheelAnchor);
			CreateLeg(1.0f, wheelAnchor);
		}

		public override void Step(TestSettings settings)
		{
			m_debugDraw.DrawString("Keys: left = a, brake = s, right = d, toggle motor = m");
			

			base.Step(settings);
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.A)) {
				m_motorJoint.SetMotorSpeed(-m_motorSpeed);
			}

			if (KeyboardManager.IsPressed(Key.A)){
				m_motorJoint.SetMotorSpeed(0.0f);
			}

			if (KeyboardManager.IsPressed(Key.A)){
				m_motorJoint.SetMotorSpeed(m_motorSpeed);
			}

			if (KeyboardManager.IsPressed(Key.A)){
				m_motorJoint.EnableMotor(!m_motorJoint.IsMotorEnabled());
			}
		}

		public static Test Create()
		{
			return new TheoJansen();
		}

		Vec2 m_offset;
		Body m_chassis;
		Body m_wheel;
		RevoluteJoint m_motorJoint;
		bool m_motorOn;
		float m_motorSpeed;
	};
}
