using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	/// This test shows how a rope joint can be used to stabilize a chain of
	/// bodies with a heavy payload. Notice that the rope joint just prevents
	/// excessive stretching and has no other effect.
	/// By disabling the rope joint you can see that the Box2D solver has trouble
	/// supporting heavy bodies with light bodies. Try playing around with the
	/// densities, time step, and iterations to see how they affect stability.
	/// This test also shows how to use contact filtering. Filtering is configured
	/// so that the payload does not collide with the chain.
	class RopeJoint : Test
	{
		public RopeJoint()
		{
			b2Body* ground = null;
			{
				b2BodyDef bd;
				ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				shape.SetAsBox(0.5f, 0.125f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 20.0f;
				fd.friction = 0.2f;
				fd.filter.categoryBits = 0x0001;
				fd.filter.maskBits = 0xFFFF & ~0x0002;

				b2RevoluteJointDef jd;
				jd.collideConnected = false;

				const int N = 10;
				const float y = 15.0f;
				m_ropeDef.localAnchorA.Set(0.0f, y);

				b2Body* prevBody = ground;
				for (int i = 0; i < N; ++i)
				{
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position.Set(0.5f + 1.0f * i, y);
					if (i == N - 1)
					{
						shape.SetAsBox(1.5f, 1.5f);
						fd.density = 100.0f;
						fd.filter.categoryBits = 0x0002;
						bd.position.Set(1.0f * i, y);
						bd.angularDamping = 0.4f;
					}

					b2Body* body = m_world.CreateBody(&bd);

					body.CreateFixture(&fd);

					b2Vec2 anchor(float(i), y);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(&jd);

					prevBody = body;
				}

				m_ropeDef.localAnchorB.SetZero();

				float extraLength = 0.01f;
				m_ropeDef.maxLength = N - 1.0f + extraLength;
				m_ropeDef.bodyB = prevBody;
			}

			{
				m_ropeDef.bodyA = ground;
				m_rope = m_world.CreateJoint(&m_ropeDef);
			}
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 'j':
				if (m_rope)
				{
					m_world.DestroyJoint(m_rope);
					m_rope = null;
				}
				else
				{
					m_rope = m_world.CreateJoint(&m_ropeDef);
				}
				break;
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Press (j) to toggle the rope joint.");
			m_textLine += DRAW_STRING_NEW_LINE;
			if (m_rope)
			{
				m_debugDraw.DrawString(5, m_textLine, "Rope ON");
			}
			else
			{
				m_debugDraw.DrawString(5, m_textLine, "Rope OFF");
			}
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		public static Test Create()
		{
			return new RopeJoint();
		}

		b2RopeJointDef m_ropeDef;
		b2Joint* m_rope;
	};
}
