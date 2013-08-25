using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class PolyCollision : Test
	{
	public:
		PolyCollision()
		{
			{
				m_polygonA.SetAsBox(0.2f, 0.4f);
				m_transformA.Set(b2Vec2(0.0f, 0.0f), 0.0f);
			}

			{
				m_polygonB.SetAsBox(0.5f, 0.5f);
				m_positionB.Set(19.345284f, 1.5632932f);
				m_angleB = 1.9160721f;
				m_transformB.Set(m_positionB, m_angleB);
			}
		}

		static Test* Create()
		{
			return new PolyCollision;
		}

		void Step(Settings* settings)
		{
			B2_NOT_USED(settings);

			b2Manifold manifold;
			b2CollidePolygons(&manifold, &m_polygonA, m_transformA, &m_polygonB, m_transformB);

			b2WorldManifold worldManifold;
			worldManifold.Initialize(&manifold, m_transformA, m_polygonA.m_radius, m_transformB, m_polygonB.m_radius);

			m_debugDraw.DrawString(5, m_textLine, "point count = %d", manifold.pointCount);
			m_textLine += DRAW_STRING_NEW_LINE;

			{
				b2Color color(0.9f, 0.9f, 0.9f);
				b2Vec2 v[b2Settings.b2_maxPolygonVertices];
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

			for (int i = 0; i < manifold.pointCount; ++i)
			{
				m_debugDraw.DrawPoint(worldManifold.points[i], 4.0f, b2Color(0.9f, 0.3f, 0.3f));
			}
		}

		void Keyboard(unsigned char key)
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
				m_angleB += 0.1f * Math.PI;
				break;

			case 'e':
				m_angleB -= 0.1f * Math.PI;
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
