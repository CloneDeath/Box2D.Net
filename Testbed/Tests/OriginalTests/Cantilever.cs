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
		const int e_count = 8;

		Cantilever()
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

				WeldJointDef jd = new WeldJointDef();

				Body prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-14.5f + 1.0f * i, 5.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					Vec2 anchor = new Vec2(-15.0f + 1.0f * i, 5.0f);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(jd);

					prevBody = body;
				}
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(1.0f, 0.125f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;

				WeldJointDef jd = new WeldJointDef();
				jd.frequencyHz = 5.0f;
				jd.dampingRatio = 0.7f;

				Body prevBody = ground;
				for (int i = 0; i < 3; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-14.0f + 2.0f * i, 15.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					Vec2 anchor = new Vec2(-15.0f + 2.0f * i, 15.0f);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(jd);

					prevBody = body;
				}
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.125f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;

				WeldJointDef jd = new WeldJointDef();

				Body prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-4.5f + 1.0f * i, 5.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					if (i > 0)
					{
						Vec2 anchor = new Vec2(-5.0f + 1.0f * i, 5.0f);
						jd.Initialize(prevBody, body, anchor);
						m_world.CreateJoint(jd);
					}

					prevBody = body;
				}
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.125f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;

				WeldJointDef jd = new WeldJointDef();
				jd.frequencyHz = 8.0f;
				jd.dampingRatio = 0.7f;

				Body prevBody = ground;
				for (int i = 0; i < e_count; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(5.5f + 1.0f * i, 10.0f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);

					if (i > 0)
					{
						Vec2 anchor = new Vec2(5.0f + 1.0f * i, 10.0f);
						jd.Initialize(prevBody, body, anchor);
						m_world.CreateJoint(jd);
					}

					prevBody = body;
				}
			}

			for (int i = 0; i < 2; ++i)
			{
				Vec2[] vertices = new Vec2[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);

				PolygonShape shape = new PolygonShape();
				shape.Set(vertices, 3);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 1.0f;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(-8.0f + 8.0f * i, 12.0f);
				Body body = m_world.CreateBody(bd);
				body.CreateFixture(fd);
			}

			for (int i = 0; i < 2; ++i)
			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 0.5f;

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 1.0f;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(-6.0f + 6.0f * i, 10.0f);
				Body body = m_world.CreateBody(bd);
				body.CreateFixture(fd);
			}
		}

		public static Test Create()
		{
			return new Cantilever();
		}

		Body m_middle;
	};
}
