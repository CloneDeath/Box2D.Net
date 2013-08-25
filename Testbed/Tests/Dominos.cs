using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Dominos : Test
	{
		public Dominos()
		{
			Body b1;
			{
				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));

				BodyDef bd = new BodyDef();
				b1 = m_world.CreateBody(bd);
				b1.CreateFixture(shape, 0.0f);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(6.0f, 0.25f);

				BodyDef bd = new BodyDef();
				bd.position.Set(-1.5f, 10.0f);
				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(shape, 0.0f);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.1f, 1.0f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.density = 20.0f;
				fd.friction = 0.1f;

				for (int i = 0; i < 10; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.position.Set(-6.0f + 1.0f * i, 11.25f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(fd);
				}
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(7.0f, 0.25f, new Vec2(0, 0), 0.3f);

				BodyDef bd = new BodyDef();
				bd.position.Set(1.0f, 6.0f);
				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(shape, 0.0f);
			}

			Body b2;
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.25f, 1.5f);

				BodyDef bd = new BodyDef();
				bd.position.Set(-7.0f, 4.0f);
				b2 = m_world.CreateBody(bd);
				b2.CreateFixture(shape, 0.0f);
			}

			Body b3;
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(6.0f, 0.125f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-0.9f, 1.0f);
				bd.angle = -0.15f;

				b3 = m_world.CreateBody(bd);
				b3.CreateFixture(shape, 10.0f);
			}

			RevoluteJointDef jd = new RevoluteJointDef();
			Vec2 anchor = new Vec2();

			anchor.Set(-2.0f, 1.0f);
			jd.Initialize(b1, b3, anchor);
			jd.collideConnected = true;
			m_world.CreateJoint(jd);

			Body b4;
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.25f, 0.25f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-10.0f, 15.0f);
				b4 = m_world.CreateBody(bd);
				b4.CreateFixture(shape, 10.0f);
			}

			anchor.Set(-7.0f, 15.0f);
			jd.Initialize(b2, b4, anchor);
			m_world.CreateJoint(jd);

			Body b5;
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(6.5f, 3.0f);
				b5 = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				FixtureDef fd = new FixtureDef();

				fd.shape = shape;
				fd.density = 10.0f;
				fd.friction = 0.1f;

				shape.SetAsBox(1.0f, 0.1f, new Vec2(0.0f, -0.9f), 0.0f);
				b5.CreateFixture(fd);

				shape.SetAsBox(0.1f, 1.0f, new Vec2(-0.9f, 0.0f), 0.0f);
				b5.CreateFixture(fd);

				shape.SetAsBox(0.1f, 1.0f, new Vec2(0.9f, 0.0f), 0.0f);
				b5.CreateFixture(fd);
			}

			anchor.Set(6.0f, 2.0f);
			jd.Initialize(b1, b5, anchor);
			m_world.CreateJoint(jd);

			Body b6;
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(1.0f, 0.1f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(6.5f, 4.1f);
				b6 = m_world.CreateBody(bd);
				b6.CreateFixture(shape, 30.0f);
			}

			anchor.Set(7.5f, 4.0f);
			jd.Initialize(b5, b6, anchor);
			m_world.CreateJoint(jd);

			Body b7;
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.1f, 1.0f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(7.4f, 1.0f);

				b7 = m_world.CreateBody(bd);
				b7.CreateFixture(shape, 10.0f);
			}

			DistanceJointDef djd = new DistanceJointDef();
			djd.bodyA = b3;
			djd.bodyB = b7;
			djd.localAnchorA.Set(6.0f, 0.0f);
			djd.localAnchorB.Set(0.0f, -1.0f);
			Vec2 d = djd.bodyB.GetWorldPoint(djd.localAnchorB) - djd.bodyA.GetWorldPoint(djd.localAnchorA);
			djd.length = d.Length();
			m_world.CreateJoint(djd);

			{
				float radius = 0.2f;

				CircleShape shape = new CircleShape();
				shape.m_radius = radius;

				for (int i = 0; i < 4; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.position.Set(5.9f + 2.0f * radius * i, 2.4f);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 10.0f);
				}
			}
		}

		public static Test Create()
		{
			return new Dominos();
		}
	};
}
