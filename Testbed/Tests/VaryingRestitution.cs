﻿using System;
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
				b2BodyDef bd = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				b2CircleShape shape = new b2CircleShape();
				shape.m_radius = 1.0f;

				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = shape;
				fd.density = 1.0f;

				float[] restitution = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

				for (int i = 0; i < 7; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(-10.0f + 3.0f * i, 20.0f);

					b2Body body = m_world.CreateBody(bd);

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
