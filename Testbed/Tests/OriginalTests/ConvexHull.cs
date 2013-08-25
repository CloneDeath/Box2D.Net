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
		const int e_count = Settings._maxPolygonVertices;

		public ConvexHull()
		{
			Generate();
			m_auto = false;
		}

		public void Generate()
		{
			Vec2 lowerBound = new Vec2(-8.0f, -8.0f);
			Vec2 upperBound = new Vec2(8.0f, 8.0f);

			for (int i = 0; i < e_count; ++i)
			{
				float x = 10.0f * RandomFloat();
				float y = 10.0f * RandomFloat();

				// Clamp onto a square to help create collinearities.
				// This will stress the convex hull algorithm.
				Vec2 v = new Vec2(x, y);
				v = Utilities.Clamp(v, lowerBound, upperBound);
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

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			PolygonShape shape = new PolygonShape();
			shape.Set(m_points, e_count);

			m_debugDraw.DrawString("Press g to generate a new random convex hull");
			

			m_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, Color.FromArgb(225, 225, 225));

			for (int i = 0; i < e_count; ++i)
			{
				m_debugDraw.DrawPoint(m_points[i], 2.0f, Color.FromArgb(225, 128, 128));
				//m_debugDraw.DrawString(m_points[i] + new Vec2(0.05f, 0.05f), "%d", i);
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

		Vec2[] m_points = new Vec2[Settings._maxPolygonVertices];
		bool m_auto;
	};
}
