using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Pyramid : Test
	{
		const int e_count = 20;

		public Pyramid()
		{
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				float a = 0.5f;
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(a, a);

				Vec2 x = new Vec2(-7.0f, 0.75f);
				Vec2 y;
				Vec2 deltaX = new Vec2(0.5625f, 1.25f);
				Vec2 deltaY = new Vec2(1.125f, 0.0f);

				for (int i = 0; i < e_count; ++i)
				{
					y = x;

					for (int j = i; j < e_count; ++j)
					{
						BodyDef bd = new BodyDef();
						bd.type = BodyType._dynamicBody;
						bd.position = y;
						Body body = m_world.CreateBody(bd);
						body.CreateFixture(shape, 5.0f);

						y += deltaY;
					}

					x += deltaX;
				}
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			//DynamicTree tree = m_world.m_contactManager.m_broadPhase.m_tree;

			//if (m_stepCount == 400)
			//{
			//	tree.RebuildBottomUp();
			//}
		}

		public static Test Create()
		{
			return new Pyramid();
		}
	};
}
