using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class EdgeTest : Test
	{
		public EdgeTest()
		{
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				Vec2 v1 = new Vec2(-10.0f, 0.0f), v2 = new Vec2(-7.0f, -2.0f), v3 = new Vec2(-4.0f, 0.0f);
				Vec2 v4 = new Vec2(0.0f, 0.0f), v5 = new Vec2(4.0f, 0.0f), v6 = new Vec2(7.0f, 2.0f), v7 = new Vec2(10.0f, 0.0f);

				EdgeShape shape = new EdgeShape();

				shape.Set(v1, v2);
				shape.m_hasVertex3 = true;
				shape.m_vertex3 = v3;
				ground.CreateFixture(shape, 0.0f);

				shape.Set(v2, v3);
				shape.m_hasVertex0 = true;
				shape.m_hasVertex3 = true;
				shape.m_vertex0 = v1;
				shape.m_vertex3 = v4;
				ground.CreateFixture(shape, 0.0f);

				shape.Set(v3, v4);
				shape.m_hasVertex0 = true;
				shape.m_hasVertex3 = true;
				shape.m_vertex0 = v2;
				shape.m_vertex3 = v5;
				ground.CreateFixture(shape, 0.0f);

				shape.Set(v4, v5);
				shape.m_hasVertex0 = true;
				shape.m_hasVertex3 = true;
				shape.m_vertex0 = v3;
				shape.m_vertex3 = v6;
				ground.CreateFixture(shape, 0.0f);

				shape.Set(v5, v6);
				shape.m_hasVertex0 = true;
				shape.m_hasVertex3 = true;
				shape.m_vertex0 = v4;
				shape.m_vertex3 = v7;
				ground.CreateFixture(shape, 0.0f);

				shape.Set(v6, v7);
				shape.m_hasVertex0 = true;
				shape.m_vertex0 = v5;
				ground.CreateFixture(shape, 0.0f);
			}

			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-0.5f, 0.6f);
				bd.allowSleep = false;
				Body body = m_world.CreateBody(bd);

				CircleShape shape = new CircleShape();
				shape.m_radius = 0.5f;

				body.CreateFixture(shape, 1.0f);
			}

			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(1.0f, 0.6f);
				bd.allowSleep = false;
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);

				body.CreateFixture(shape, 1.0f);
			}
		}

		public static Test Create()
		{
			return new EdgeTest();
		}
	};
}
