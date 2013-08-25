using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// It is difficult to make a cantilever made of links completely rigid with weld joints.
	// You will have to use a high number of iterations to make them stiff.
	// So why not go ahead and use soft weld joints? They behave like a revolute
	// joint with a rotational spring.
	class Cantilever : Test
	{
	public enum
		{
			e_count = 8
		};

		Cantilever()
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
				shape.SetAsBox(0.5f, 0.125f);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.density = 20.0f;

				b2WeldJointDef jd;

				b2Body prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(-14.5f + 1.0f * i, 5.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					b2Vec2 anchor(-15.0f + 1.0f * i, 5.0f);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(&jd);

					prevBody = body;
				}
			}

			{
				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(1.0f, 0.125f);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.density = 20.0f;

				b2WeldJointDef jd;
				jd.frequencyHz = 5.0f;
				jd.dampingRatio = 0.7f;

				b2Body prevBody = ground;
				for (int i = 0; i < 3; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(-14.0f + 2.0f * i, 15.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					b2Vec2 anchor(-15.0f + 2.0f * i, 15.0f);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(&jd);

					prevBody = body;
				}
			}

			{
				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.5f, 0.125f);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.density = 20.0f;

				b2WeldJointDef jd;

				b2Body prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(-4.5f + 1.0f * i, 5.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					if (i > 0)
					{
						b2Vec2 anchor(-5.0f + 1.0f * i, 5.0f);
						jd.Initialize(prevBody, body, anchor);
						m_world.CreateJoint(&jd);
					}

					prevBody = body;
				}
			}

			{
				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.5f, 0.125f);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.density = 20.0f;

				b2WeldJointDef jd;
				jd.frequencyHz = 8.0f;
				jd.dampingRatio = 0.7f;

				b2Body prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					b2BodyDef bd = new b2BodyDef();
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position.Set(5.5f + 1.0f * i, 10.0f);
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					if (i > 0)
					{
						b2Vec2 anchor(5.0f + 1.0f * i, 10.0f);
						jd.Initialize(prevBody, body, anchor);
						m_world.CreateJoint(&jd);
					}

					prevBody = body;
				}
			}

			for (int i = 0; i < 2; ++i)
			{
				b2Vec2 vertices[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);

				b2PolygonShape shape = new b2PolygonShape();
				shape.Set(vertices, 3);

				b2FixtureDef fd;
				fd.shape = shape;
				fd.density = 1.0f;

				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(-8.0f + 8.0f * i, 12.0f);
				b2Body body = m_world.CreateBody(bd);
				body.CreateFixture(fd);
			}

			for (int i = 0; i < 2; ++i)
			{
				b2CircleShape shape = new b2CircleShape();
				shape.m_radius = 0.5f;

				b2FixtureDef fd;
				fd.shape = shape;
				fd.density = 1.0f;

				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(-6.0f + 6.0f * i, 10.0f);
				b2Body body = m_world.CreateBody(bd);
				body.CreateFixture(fd);
			}
		}

		public static Test Create()
		{
			return new Cantilever();
		}

		b2Body m_middle;
	};
}
