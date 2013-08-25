using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	/// This test shows how a rope joint can be used to stabilize a chain of
	/// bodies with a heavy payload. Notice that the rope joint just prevents
	/// excessive stretching and has no other effect.
	/// By disabling the rope joint you can see that the Box2D solver has trouble
	/// supporting heavy bodies with light bodies. Try playing around with the
	/// densities, time step, and iterations to see how they affect stability.
	/// This test also shows how to use contact filtering. Filtering is configured
	/// so that the payload does not collide with the chain.
	class RopeJointTest : Test
	{
		public RopeJointTest()
		{
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.125f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				fd.friction = 0.2f;
				fd.Filter.CategoryBits = 0x0001;
				fd.Filter.MaskBits = 0xFFFF & ~0x0002;

				RevoluteJointDef jd = new RevoluteJointDef();
				jd.collideConnected = false;

				const int N = 10;
				const float y = 15.0f;
				m_ropeDef.localAnchorA.Set(0.0f, y);

				Body prevBody = ground;
				for (int i = 0; i < N; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(0.5f + 1.0f * i, y);
					if (i == N - 1)
					{
						shape.SetAsBox(1.5f, 1.5f);
						fd.Density = 100.0f;
						fd.Filter.CategoryBits = 0x0002;
						bd.Position.Set(1.0f * i, y);
						bd.angularDamping = 0.4f;
					}

					Body body = m_world.CreateBody(bd);

					body.CreateFixture(fd);

					Vec2 anchor = new Vec2((float)(i), y);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(jd);

					prevBody = body;
				}

				m_ropeDef.localAnchorB.SetZero();

				float extraLength = 0.01f;
				m_ropeDef.maxLength = N - 1.0f + extraLength;
				m_ropeDef.bodyB = prevBody;
			}

			{
				m_ropeDef.bodyA = ground;
				m_rope = m_world.CreateJoint(m_ropeDef);
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.J)) {
				if (m_rope != null)
				{
					m_world.DestroyJoint(m_rope);
					m_rope = null;
				}
				else
				{
					m_rope = m_world.CreateJoint(m_ropeDef);
				}
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Press (j) to toggle the rope joint.");
			
			if (m_rope != null)
			{
				m_debugDraw.DrawString("Rope ON");
			}
			else
			{
				m_debugDraw.DrawString("Rope OFF");
			}
			
		}

		public static Test Create()
		{
			return new RopeJointTest();
		}

		RopeJointDef m_ropeDef;
		Joint m_rope;
	};
}
