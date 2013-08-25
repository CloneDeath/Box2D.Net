using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;
using System;

namespace Testbed.Tests.BasicTests
{
	class ApplyForce : Test
	{
		Body _body;

		public ApplyForce()
		{
			m_world.Gravity = new Vec2(0.0f, 0.0f);

			const float k_restitution = 0.4f;

			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, 20.0f);
				Body ground = m_world.CreateBody(bd);

				PolygonDef sd = new PolygonDef();
				sd.Density = 0.0f;
				sd.Restitution = k_restitution;

				sd.SetAsBox(0.2f, 20.0f, new Vec2(-20.0f, 0.0f), 0.0f);
				ground.CreateFixture(sd);

				sd.SetAsBox(0.2f, 20.0f, new Vec2(20.0f, 0.0f), 0.0f);
				ground.CreateFixture(sd);

				sd.SetAsBox(0.2f, 20.0f, new Vec2(0.0f, -20.0f), 0.5f * (float)Math.PI);
				ground.CreateFixture(sd);

				sd.SetAsBox(0.2f, 20.0f, new Vec2(0.0f, 20.0f), -0.5f * (float)Math.PI);
				ground.CreateFixture(sd);
			}

			{
				Transform xf1 = new Transform();
				xf1.q.Set(0.3524f * (float)Math.PI);
				xf1.p = Utilities.Mul(xf1.q, new Vec2(1.0f, 0.0f));

				PolygonDef sd1 = new PolygonDef();
				sd1.Vertices = new Vec2[]{
					Utilities.Mul(xf1, new Vec2(-1.0f, 0.0f)),
					Utilities.Mul(xf1, new Vec2(1.0f, 0.0f)),
					Utilities.Mul(xf1, new Vec2(0.0f, 0.5f)),
				};
				sd1.Density = 2.0f;

				Transform xf2 = new Transform();
				xf2.q.Set(-0.3524f * (float)Math.PI);
				xf2.p = Utilities.Mul(xf2.q, new Vec2(-1.0f, 0.0f));

				PolygonDef sd2 = new PolygonDef();
				sd2.Vertices = new Vec2[]{
					Utilities.Mul(xf2, new Vec2(-1.0f, 0.0f)),
					Utilities.Mul(xf2, new Vec2(1.0f, 0.0f)),
					Utilities.Mul(xf2, new Vec2(0.0f, 0.5f)),
				};
				sd2.Density = 2.0f;

				BodyDef bd = new BodyDef();
				bd.angularDamping = 2.0f;
				bd.linearDamping = 0.1f;

				bd.Position.Set(0.0f, 1.05f);
				bd.angle = (float)Math.PI;
				_body = m_world.CreateBody(bd);
				_body.CreateFixture(sd1);
				_body.CreateFixture(sd2);
				_body.SetMassFromShapes();
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.W)){
				Vec2 f = _body.GetWorldVector(new Vec2(0.0f, -200.0f));
				Vec2 p = _body.GetWorldPoint(new Vec2(0.0f, 2.0f));
				_body.ApplyForce(f, p);
			}

			if (KeyboardManager.IsPressed(Key.A)){
				_body.ApplyTorque(20.0f);
			}
			if (KeyboardManager.IsPressed(Key.D)){
				_body.ApplyTorque(-20.0f);
			}
		}

		public static Test Create()
		{
			return new ApplyForce();
		}
	}
}