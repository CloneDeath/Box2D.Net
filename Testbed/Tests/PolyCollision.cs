using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;

namespace Testbed.Tests {
	class PolyCollision : Test
	{
		public PolyCollision()
		{
			{
				m_polygonA.SetAsBox(0.2f, 0.4f);
				m_transformA.Set(new b2Vec2(0.0f, 0.0f), 0.0f);
			}

			{
				m_polygonB.SetAsBox(0.5f, 0.5f);
				m_positionB.Set(19.345284f, 1.5632932f);
				m_angleB = 1.9160721f;
				m_transformB.Set(m_positionB, m_angleB);
			}
		}

		public static Test Create()
		{
			return new PolyCollision();
		}

		public override void Step(Settings settings)
		{
			b2Manifold manifold;
			b2Collision.b2CollidePolygons(out manifold, m_polygonA, m_transformA, m_polygonB, m_transformB);

			b2WorldManifold worldManifold = new b2WorldManifold();
			worldManifold.Initialize(manifold, m_transformA, m_polygonA.m_radius, m_transformB, m_polygonB.m_radius);

			m_debugDraw.DrawString("point count = {0}", manifold.points.Count());
			

			{
				Color color = Color.FromArgb(225, 225, 225);
				b2Vec2[] v = new b2Vec2[b2Settings.b2_maxPolygonVertices];
				for (int i = 0; i < m_polygonA.m_count; ++i)
				{
					v[i] = Utilities.b2Mul(m_transformA, m_polygonA.m_vertices[i]);
				}
				m_debugDraw.DrawPolygon(v, m_polygonA.m_count, color);

				for (int i = 0; i < m_polygonB.m_count; ++i)
				{
					v[i] = Utilities.b2Mul(m_transformB, m_polygonB.m_vertices[i]);
				}
				m_debugDraw.DrawPolygon(v, m_polygonB.m_count, color);
			}

			for (int i = 0; i < manifold.points.Count(); ++i)
			{
				m_debugDraw.DrawPoint(worldManifold.points[i], 4.0f, Color.FromArgb(225, 75, 75));
			}
		}

		public override void Keyboard()
		{
			switch (key)
			{
			case 'a':
				m_positionB.x -= 0.1f;
				break;

			case 'd':
				m_positionB.x += 0.1f;
				break;

			case 's':
				m_positionB.y -= 0.1f;
				break;

			case 'w':
				m_positionB.y += 0.1f;
				break;

			case 'q':
				m_angleB += 0.1f * (float)Math.PI;
				break;

			case 'e':
				m_angleB -= 0.1f * (float)Math.PI;
				break;
			}

			m_transformB.Set(m_positionB, m_angleB);
		}

		b2PolygonShape m_polygonA;
		b2PolygonShape m_polygonB;

		b2Transform m_transformA;
		b2Transform m_transformB;

		b2Vec2 m_positionB;
		float m_angleB;
	};
}
