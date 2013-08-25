using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// TODO_ERIN test joints on compounds.
	class CompoundShapes : Test
	{
		public CompoundShapes()
		{
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, 0.0f);
				Body body = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(50.0f, 0.0f), new Vec2(-50.0f, 0.0f));
				shape.Density = 0;
				body.CreateFixture(shape);
			}

			{
				CircleShape circle1 = new CircleShape();
				circle1.m_radius = 0.5f;
				circle1.m_p.Set(-0.5f, 0.5f);
				circle1.Density = 2;

				CircleShape circle2 = new CircleShape();
				circle2.m_radius = 0.5f;
				circle2.m_p.Set(0.5f, 0.5f);
				circle2.Density = 0;

				for (int i = 0; i < 10; ++i)
				{
					float x = RandomFloat(-0.1f, 0.1f);
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(x + 5.0f, 1.05f + 2.5f * i);
					bd.angle = RandomFloat(-(float)Math.PI, (float)Math.PI);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(circle1);
					body.CreateFixture(circle2);
				}
			}

			{
				PolygonShape polygon1 = new PolygonShape();
				polygon1.SetAsBox(0.25f, 0.5f);
				polygon1.Density = 2;

				PolygonShape polygon2 = new PolygonShape();
				polygon2.SetAsBox(0.25f, 0.5f, new Vec2(0.0f, -0.5f), 0.5f * (float)Math.PI);
				polygon2.Density = 2;

				for (int i = 0; i < 10; ++i)
				{
					float x = RandomFloat(-0.1f, 0.1f);
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(x - 5.0f, 1.05f + 2.5f * i);
					bd.angle = RandomFloat(-(float)Math.PI, (float)Math.PI);
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(polygon1);
					body.CreateFixture(polygon2);
				}
			}

			{
				Transform xf1 = new Transform();
				xf1.q.Set(0.3524f * (float)Math.PI);
				xf1.p = xf1.q.GetXAxis();

				Vec2[] vertices = new Vec2[3];

				PolygonShape triangle1 = new PolygonShape();
				vertices[0] = Utilities.Mul(xf1, new Vec2(-1.0f, 0.0f));
				vertices[1] = Utilities.Mul(xf1, new Vec2(1.0f, 0.0f));
				vertices[2] = Utilities.Mul(xf1, new Vec2(0.0f, 0.5f));
				triangle1.Set(vertices, 3);

				Transform xf2 = new Transform();
				xf2.q.Set(-0.3524f * (float)Math.PI);
				xf2.p = -xf2.q.GetXAxis();

				PolygonShape triangle2 = new PolygonShape();
				vertices[0] = Utilities.Mul(xf2, new Vec2(-1.0f, 0.0f));
				vertices[1] = Utilities.Mul(xf2, new Vec2(1.0f, 0.0f));
				vertices[2] = Utilities.Mul(xf2, new Vec2(0.0f, 0.5f));
				triangle2.Set(vertices, 3);

				for (int i = 0; i < 10; ++i)
				{
					float x = RandomFloat(-0.1f, 0.1f);
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(x, 2.05f + 2.5f * i);
					bd.angle = 0.0f;
					Body body = m_world.CreateBody(bd);
					triangle1.Density = 2;
					triangle2.Density = 2;
					body.CreateFixture(triangle1);
					body.CreateFixture(triangle2);
				}
			}

			{
				PolygonShape bottom = new PolygonShape();
				bottom.SetAsBox( 1.5f, 0.15f );

				PolygonShape left = new PolygonShape();
				left.SetAsBox(0.15f, 2.7f, new Vec2(-1.45f, 2.35f), 0.2f);

				PolygonShape right = new PolygonShape();
				right.SetAsBox(0.15f, 2.7f, new Vec2(1.45f, 2.35f), -0.2f);

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set( 0.0f, 2.0f );
				Body body = m_world.CreateBody(bd);
				bottom.Density = 4;
				left.Density = 4;
				right.Density = 4;
				body.CreateFixture(bottom);
				body.CreateFixture(left);
				body.CreateFixture(right);
			}
		}

		public static Test Create()
		{
			return new CompoundShapes();
		}
	};
}
