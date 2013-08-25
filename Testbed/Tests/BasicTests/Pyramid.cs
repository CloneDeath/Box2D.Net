using Testbed.Framework;
using Box2D;

namespace Testbed.Tests.BasicTests
{
	class Pyramid : Test
	{
		public Pyramid()
		{
			{
				PolygonDef sd = new PolygonDef();
				sd.SetAsBox(50.0f, 10.0f);

				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, -10.0f);
				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(sd);
			}

			{
				PolygonDef sd = new PolygonDef();
				float a = 0.5f;
				sd.SetAsBox(a, a);
				sd.Density = 5.0f;

				Vec2 x = new Vec2(-10.0f, 0.75f);
				Vec2 y;
				Vec2 deltaX = new Vec2(0.5625f, 2.0f);
				Vec2 deltaY = new Vec2(1.125f, 0.0f);

				for (int i = 0; i < 25; ++i)
				{
					y = x;

					for (int j = i; j < 25; ++j)
					{
						BodyDef bd = new BodyDef();
						bd.Position = y;
						Body body = m_world.CreateBody(bd);
						body.CreateFixture(sd);
						body.SetMassFromShapes();

						y += deltaY;
					}

					x += deltaX;
				}
			}
		}

		public static Test Create()
		{
			return new Pyramid();
		}
	}
}
