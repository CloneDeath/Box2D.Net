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
				b2BodyDef bd = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd);

				b2Vec2 v1(-10.0f, 0.0f), v2(-7.0f, -2.0f), v3(-4.0f, 0.0f);
				b2Vec2 v4(0.0f, 0.0f), v5(4.0f, 0.0f), v6(7.0f, 2.0f), v7(10.0f, 0.0f);

				b2EdgeShape shape = new b2EdgeShape();

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
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(-0.5f, 0.6f);
				bd.allowSleep = false;
				b2Body body = m_world.CreateBody(bd);

				b2CircleShape shape = new b2CircleShape();
				shape.m_radius = 0.5f;

				body.CreateFixture(shape, 1.0f);
			}

			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(1.0f, 0.6f);
				bd.allowSleep = false;
				b2Body body = m_world.CreateBody(bd);

				b2PolygonShape shape = new b2PolygonShape();
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
