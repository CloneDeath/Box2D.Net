using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class ConvexHull : Test
	{
		public enum
		{
			e_count = b2Settings.b2_maxPolygonVertices
		};

		public ConvexHull()
		{
			Generate();
			m_auto = false;
		}

		public void Generate()
		{
			b2Vec2 lowerBound(-8.0f, -8.0f);
			b2Vec2 upperBound(8.0f, 8.0f);

			for (int i = 0; i < e_count; ++i)
			{
				float x = 10.0f * RandomFloat();
				float y = 10.0f * RandomFloat();

				// Clamp onto a square to help create collinearities.
				// This will stress the convex hull algorithm.
				b2Vec2 v(x, y);
				v = b2Clamp(v, lowerBound, upperBound);
				m_points[i] = v;
			}
		}

		public static Test Create()
		{
			return new ConvexHull();
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 'a':
				m_auto = !m_auto;
				break;

			case 'g':
				Generate();
				break;
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			b2PolygonShape shape = new b2PolygonShape();
			shape.Set(m_points, e_count);

			m_debugDraw.DrawString("Press g to generate a new random convex hull");
			

			m_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, b2Color(0.9f, 0.9f, 0.9f));

			for (int i = 0; i < e_count; ++i)
			{
				m_debugDraw.DrawPoint(m_points[i], 2.0f, b2Color(0.9f, 0.5f, 0.5f));
				m_debugDraw.DrawString(m_points[i] + b2Vec2(0.05f, 0.05f), "%d", i);
			}

			if (shape.Validate() == false)
			{
				m_textLine += 0;
			}

			if (m_auto)
			{
				Generate();
			}
		}

		b2Vec2 m_points[b2Settings.b2_maxPolygonVertices];
		bool m_auto;
	};
}
