using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class ApplyForce : Test {
		Body m_body;

		public ApplyForce() {
			m_world.SetGravity(new Vec2(0.0f, 0.0f));

			const float k_restitution = 0.4f;

			Body ground;
			{
				BodyDef bd = new BodyDef();
				bd.position.Set(0.0f, 20.0f);
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();

				FixtureDef sd = new FixtureDef();
				sd.shape = shape;
				sd.density = 0.0f;
				sd.restitution = k_restitution;

				// Left vertical
				shape.Set(new Vec2(-20.0f, -20.0f), new Vec2(-20.0f, 20.0f));
				ground.CreateFixture(sd);

				// Right vertical
				shape.Set(new Vec2(20.0f, -20.0f), new Vec2(20.0f, 20.0f));
				ground.CreateFixture(sd);

				// Top horizontal
				shape.Set(new Vec2(-20.0f, 20.0f), new Vec2(20.0f, 20.0f));
				ground.CreateFixture(sd);

				// Bottom horizontal
				shape.Set(new Vec2(-20.0f, -20.0f), new Vec2(20.0f, -20.0f));
				ground.CreateFixture(sd);
			}

			{
				Transform xf1 = new Transform();
				xf1.q.Set(0.3524f * (float)Math.PI);
				xf1.p = xf1.q.GetXAxis();

				Vec2[] vertices = new Vec2[3];
				vertices[0] = Utilities.Mul(xf1, new Vec2(-1.0f, 0.0f));
				vertices[1] = Utilities.Mul(xf1, new Vec2(1.0f, 0.0f));
				vertices[2] = Utilities.Mul(xf1, new Vec2(0.0f, 0.5f));

				PolygonShape poly1 = new PolygonShape();
				poly1.Set(vertices, 3);

				FixtureDef sd1 = new FixtureDef();
				sd1.shape = poly1;
				sd1.density = 4.0f;

				Transform xf2 = new Transform();
				xf2.q.Set(-0.3524f * (float)Math.PI);
				xf2.p = -xf2.q.GetXAxis();

				vertices[0] = Utilities.Mul(xf2, new Vec2(-1.0f, 0.0f));
				vertices[1] = Utilities.Mul(xf2, new Vec2(1.0f, 0.0f));
				vertices[2] = Utilities.Mul(xf2, new Vec2(0.0f, 0.5f));

				PolygonShape poly2 = new PolygonShape();
				poly2.Set(vertices, 3);

				FixtureDef sd2 = new FixtureDef();
				sd2.shape = poly2;
				sd2.density = 2.0f;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.angularDamping = 5.0f;
				bd.linearDamping = 0.1f;

				bd.position.Set(0.0f, 2.0f);
				bd.angle = (float)Math.PI;
				bd.allowSleep = false;
				m_body = m_world.CreateBody(bd);
				m_body.CreateFixture(sd1);
				m_body.CreateFixture(sd2);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.density = 1.0f;
				fd.friction = 0.3f;

				for (int i = 0; i < 10; ++i) {
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;

					bd.position.Set(0.0f, 5.0f + 1.54f * i);
					Body body = m_world.CreateBody(bd);

					body.CreateFixture(fd);

					float gravity = 10.0f;
					float I = body.GetInertia();
					float mass = body.GetMass();

					// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
					float radius = (float)Math.Sqrt(2.0f * I / mass);

					FrictionJointDef jd = new FrictionJointDef();
					jd.localAnchorA.SetZero();
					jd.localAnchorB.SetZero();
					jd.bodyA = ground;
					jd.bodyB = body;
					jd.collideConnected = true;
					jd.maxForce = mass * gravity;
					jd.maxTorque = mass * radius * gravity;

					m_world.CreateJoint(jd);
				}
			}
		}

		public override void Keyboard() {
			if (KeyboardManager.IsPressed(Key.W)) {

				Vec2 f = m_body.GetWorldVector(new Vec2(0.0f, -200.0f));
				Vec2 p = m_body.GetWorldPoint(new Vec2(0.0f, 2.0f));
				m_body.ApplyForce(f, p, true);
			}

			if (KeyboardManager.IsPressed(Key.A)) {

				m_body.ApplyTorque(50.0f, true);
			}

			if (KeyboardManager.IsPressed(Key.D)) {
				m_body.ApplyTorque(-50.0f, true);
			}
		}

		public static Test Create() {
			return new ApplyForce();
		}
	}
}
