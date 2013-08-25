using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class DistanceTest : Test
	{
		public DistanceTest()
		{
			{
				m_transformA.SetIdentity();
				m_transformA.p.Set(0.0f, -0.2f);
				m_polygonA.SetAsBox(10.0f, 0.2f);
			}

			{
				m_positionB.Set(12.017401f, 0.13678508f);
				m_angleB = -0.0109265f;
				m_transformB.Set(m_positionB, m_angleB);

				m_polygonB.SetAsBox(2.0f, 0.1f);
			}
		}

		public static Test Create()
		{
			return new DistanceTest();
		}

		public void Step(Settings settings)
		{
			base.Step(settings);

			b2DistanceInput input;
			input.proxyA.Set(&m_polygonA, 0);
			input.proxyB.Set(&m_polygonB, 0);
			input.transformA = m_transformA;
			input.transformB = m_transformB;
			input.useRadii = true;
			b2SimplexCache cache;
			cache.count = 0;
			b2DistanceOutput output;
			b2Distance(&output, &cache, &input);

			m_debugDraw.DrawString("distance = %g", output.distance);
			

			m_debugDraw.DrawString("iterations = %d", output.iterations);
			

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

			b2Vec2 x1 = output.pointA;
			b2Vec2 x2 = output.pointB;

			b2Color c1(1.0f, 0.0f, 0.0f);
			m_debugDraw.DrawPoint(x1, 4.0f, c1);

			b2Color c2(1.0f, 1.0f, 0.0f);
			m_debugDraw.DrawPoint(x2, 4.0f, c2);
		}

		public void Keyboard()
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

		b2Vec2 m_positionB;
		float m_angleB;

		b2Transform m_transformA;
		b2Transform m_transformB;
		b2PolygonShape m_polygonA;
		b2PolygonShape m_polygonB;
	};
}
