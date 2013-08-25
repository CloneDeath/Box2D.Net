using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class VaryingFriction : Test
	{
		public VaryingFriction()
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
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(13.0f, 0.25f);

				BodyDef bd = new BodyDef();
				bd.Position.Set(-4.0f, 22.0f);
				bd.angle = -0.25f;

				Body ground = m_world.CreateBody(bd);
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.25f, 1.0f);
				shape.Density = 0;

				BodyDef bd = new BodyDef();
				bd.Position.Set(10.5f, 19.0f);

				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(shape);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(13.0f, 0.25f);
				shape.Density = 0;

				BodyDef bd = new BodyDef();
				bd.Position.Set(4.0f, 14.0f);
				bd.angle = 0.25f;

				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(shape);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.25f, 1.0f);
				shape.Density = 0;

				BodyDef bd = new BodyDef();
				bd.Position.Set(-10.5f, 11.0f);

				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(shape);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(13.0f, 0.25f);
				shape.Density = 0;

				BodyDef bd = new BodyDef();
				bd.Position.Set(-4.0f, 6.0f);
				bd.angle = -0.25f;

				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(shape);
			}

			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 25.0f;

				float[] friction = new float[]{ 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

				for (int i = 0; i < 5; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(-15.0f + 4.0f * i, 28.0f);
					Body body = m_world.CreateBody(bd);

					fd.friction = friction[i];
					body.CreateFixture(fd);
				}
			}
		}

		public static Test Create()
		{
			return new VaryingFriction();
		}
	};
}
