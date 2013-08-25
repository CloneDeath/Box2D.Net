using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// Note: even with a restitution of 1.0, there is some energy change
	// due to position correction.
	class VaryingRestitution : Test
	{
		public VaryingRestitution()
		{
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
				
			}

			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 1.0f;

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 1.0f;

				float[] restitution = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

				for (int i = 0; i < 7; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-10.0f + 3.0f * i, 20.0f);

					Body body = m_world.CreateBody(bd);

					fd.restitution = restitution[i];
					body.CreateFixture(fd);
				}
			}
		}

		public static Test Create()
		{
			return new VaryingRestitution();
		}
	};
}
