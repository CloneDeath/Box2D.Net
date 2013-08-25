﻿using System;
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
			b2Body ground = null;
			{
				b2BodyDef bd = new b2BodyDef();
				ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.6f, 0.125f);

				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = shape;
				fd.density = 20.0f;
				fd.friction = 0.2f;

				b2RevoluteJointDef jd = new b2RevoluteJointDef();
				jd.collideConnected = false;

				const float y = 25.0f;
				b2Body prevBody = ground;
				for (int i = 0; i < 30; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(0.5f + i, y);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					b2Vec2 anchor = new b2Vec2((float)(i), y);
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
