using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Chain : Test
	{
		public Chain()
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
				shape.SetAsBox(0.6f, 0.125f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				fd.friction = 0.2f;

				RevoluteJointDef jd = new RevoluteJointDef();
				jd.collideConnected = false;

				const float y = 25.0f;
				Body prevBody = ground;
				for (int i = 0; i < 30; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(0.5f + i, y);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					Vec2 anchor = new Vec2((float)(i), y);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(jd);

					prevBody = body;
				}
			}
		}

		public static Test Create()
		{
			return new Chain();
		}
	};
}
