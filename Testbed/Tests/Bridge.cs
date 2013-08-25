using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class Bridge : Test
	{
	public:

		enum
		{
			e_count = 30
		};

		Bridge()
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

				b2RevoluteJointDef jd;

				b2Body* prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position.Set(-14.5f + 1.0f * i, 5.0f);
					b2Body* body = m_world.CreateBody(&bd);
					body.CreateFixture(&fd);

					b2Vec2 anchor(-15.0f + 1.0f * i, 5.0f);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(&jd);

					if (i == (e_count >> 1))
					{
						m_middle = body;
					}
					prevBody = body;
				}

				b2Vec2 anchor(-15.0f + 1.0f * e_count, 5.0f);
				jd.Initialize(prevBody, ground, anchor);
				m_world.CreateJoint(&jd);
			}

			for (int i = 0; i < 2; ++i)
			{
				b2Vec2 vertices[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);

				b2PolygonShape shape;
				shape.Set(vertices, 3);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 1.0f;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-8.0f + 8.0f * i, 12.0f);
				b2Body* body = m_world.CreateBody(&bd);
				body.CreateFixture(&fd);
			}

			for (int i = 0; i < 3; ++i)
			{
				b2CircleShape shape;
				shape.m_radius = 0.5f;

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 1.0f;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-6.0f + 6.0f * i, 10.0f);
				b2Body* body = m_world.CreateBody(&bd);
				body.CreateFixture(&fd);
			}
		}

		static Test* Create()
		{
			return new Bridge;
		}

		b2Body* m_middle;
	};
}
