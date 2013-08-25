using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	/// This stress tests the dynamic tree broad-phase. This also shows that tile
	/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
	class Tiles : Test
	{
		const int e_count = 20;

		public Tiles()
		{
			m_fixtureCount = 0;
			b2Timer timer;

			{
				float a = 0.5f;
				b2BodyDef bd = new b2BodyDef();
				bd.position.y = -a;
				b2Body ground = m_world.CreateBody(bd);

	#if true
				int N = 200;
				int M = 10;
				b2Vec2 position;
				position.y = 0.0f;
				for (int j = 0; j < M; ++j)
				{
					position.x = -N * a;
					for (int i = 0; i < N; ++i)
					{
						b2PolygonShape shape = new b2PolygonShape();
						shape.SetAsBox(a, a, position, 0.0f);
						ground.CreateFixture(shape, 0.0f);
						++m_fixtureCount;
						position.x += 2.0f * a;
					}
					position.y -= 2.0f * a;
				}
	#else
				int N = 200;
				int M = 10;
				b2Vec2 position;
				position.x = -N * a;
				for (int i = 0; i < N; ++i)
				{
					position.y = 0.0f;
					for (int j = 0; j < M; ++j)
					{
						b2PolygonShape shape = new b2PolygonShape();
						shape.SetAsBox(a, a, position, 0.0f);
						ground.CreateFixture(shape, 0.0f);
						position.y -= 2.0f * a;
					}
					position.x += 2.0f * a;
				}
	#endif
			}

			{
				float a = 0.5f;
				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(a, a);

				b2Vec2 x = new b2Vec2(-7.0f, 0.75f);
				b2Vec2 y;
				b2Vec2 deltaX = new b2Vec2(0.5625f, 1.25f);
				b2Vec2 deltaY = new b2Vec2(1.125f, 0.0f);

				for (int i = 0; i < e_count; ++i)
				{
					y = x;

					for (int j = i; j < e_count; ++j)
					{
						b2BodyDef bd = new b2BodyDef();
						bd.type = b2BodyType.b2_dynamicBody;
						bd.position = y;

						//if (i == 0 && j == 0)
						//{
						//	bd.allowSleep = false;
						//}
						//else
						//{
						//	bd.allowSleep = true;
						//}

						b2Body body = m_world.CreateBody(bd);
						body.CreateFixture(shape, 5.0f);
						++m_fixtureCount;
						y += deltaY;
					}

					x += deltaX;
				}
			}

			m_createTime = timer.GetMilliseconds();
		}

		public override void Step(Settings settings)
		{
			b2ContactManager cm = m_world.GetContactManager();
			int height = cm.m_broadPhase.GetTreeHeight();
			int leafCount = cm.m_broadPhase.GetProxyCount();
			int minimumNodeCount = 2 * leafCount - 1;
			float minimumHeight = (float)Math.Ceiling(Math.Log(minimumNodeCount) / Math.Log(2.0f));
			m_debugDraw.DrawString("dynamic tree height = %d, min = %d", height, (int)minimumHeight);
			

			base.Step(settings);

			m_debugDraw.DrawString("create time = %6.2f ms, fixture count = %d",
				m_createTime, m_fixtureCount);
			

			//b2DynamicTree tree = m_world.m_contactManager.m_broadPhase.m_tree;

			//if (m_stepCount == 400)
			//{
			//	tree.RebuildBottomUp();
			//}
		}

		public static Test Create()
		{
			return new Tiles();
		}

		int m_fixtureCount;
		float m_createTime;
	};
}
