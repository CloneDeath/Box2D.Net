using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class ConvexHull : Test
	{
		const int e_count = b2Settings.b2_maxPolygonVertices;

		public ConvexHull()
		{
			Generate();
			m_auto = false;
		}

		public void Generate()
		{
			b2Vec2 lowerBound = new b2Vec2(-8.0f, -8.0f);
			b2Vec2 upperBound = new b2Vec2(8.0f, 8.0f);

			for (int i = 0; i < e_count; ++i)
			{
				float x = 10.0f * RandomFloat();
				float y = 10.0f * RandomFloat();

				// Clamp onto a square to help create collinearities.
				// This will stress the convex hull algorithm.
				b2Vec2 v = new b2Vec2(x, y);
				v = Utilities.b2Clamp(v, lowerBound, upperBound);
				m_points[i] = v;
			}
		}

		public static Test Create()
		{
			return new ConvexHull();
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.A)){
				m_auto = !m_auto;
			}
			if (KeyboardManager.IsPressed(Key.G)){

				Generate();
				
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			b2PolygonShape shape = new b2PolygonShape();
			shape.Set(m_points, e_count);

			m_debugDraw.DrawString("Press g to generate a new random convex hull");
			

			m_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, Color.FromArgb(225, 225, 225));

			for (int i = 0; i < e_count; ++i)
			{
				m_debugDraw.DrawPoint(m_points[i], 2.0f, Color.FromArgb(225, 128, 128));
				//m_debugDraw.DrawString(m_points[i] + new b2Vec2(0.05f, 0.05f), "%d", i);
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

		b2Vec2[] m_points = new b2Vec2[b2Settings.b2_maxPolygonVertices];
		bool m_auto;
	};
}
