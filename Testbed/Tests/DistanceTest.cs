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

		public void Step(TestSettings settings)
		{
			base.Step(settings);

			DistanceInput input = new DistanceInput();
			input.proxyA.Set(m_polygonA, 0);
			input.proxyB.Set(m_polygonB, 0);
			input.transformA = m_transformA;
			input.transformB = m_transformB;
			input.useRadii = true;
			SimplexCache cache = new SimplexCache();
			cache.count = 0;
			DistanceOutput output;
			Utilities.Distance(out output, cache, input);

			m_debugDraw.DrawString("distance = %g", output.distance);
			

			m_debugDraw.DrawString("iterations = %d", output.iterations);
			

			{
				Color color = Color.FromArgb(225, 225, 225);
				Vec2[] v = new Vec2[Settings._maxPolygonVertices];
				for (int i = 0; i < m_polygonA.m_count; ++i)
				{
					v[i] = Utilities.Mul(m_transformA, m_polygonA.m_vertices[i]);
				}
				m_debugDraw.DrawPolygon(v, m_polygonA.m_count, color);

				for (int i = 0; i < m_polygonB.m_count; ++i)
				{
					v[i] = Utilities.Mul(m_transformB, m_polygonB.m_vertices[i]);
				}
				m_debugDraw.DrawPolygon(v, m_polygonB.m_count, color);
			}

			Vec2 x1 = output.pointA;
			Vec2 x2 = output.pointB;

			Color c1 = Color.FromArgb(255, 0, 0);
			m_debugDraw.DrawPoint(x1, 4.0f, c1);

			Color c2 = Color.FromArgb(255, 255, 0);
			m_debugDraw.DrawPoint(x2, 4.0f, c2);
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.A)){
				m_positionB.x -= 0.1f;
			}

			if (KeyboardManager.IsPressed(Key.D)){
				m_positionB.x += 0.1f;
			}

			if (KeyboardManager.IsPressed(Key.S)){
				m_positionB.y -= 0.1f;
			}

			if (KeyboardManager.IsPressed(Key.W)){
				m_positionB.y += 0.1f;
			}

			if (KeyboardManager.IsPressed(Key.Q)){
				m_angleB += 0.1f * (float)Math.PI;
			}

			if (KeyboardManager.IsPressed(Key.E)) {
				m_angleB -= 0.1f * (float)Math.PI;
			}

			m_transformB.Set(m_positionB, m_angleB);
		}

		Vec2 m_positionB;
		float m_angleB;

		Transform m_transformA;
		Transform m_transformB;
		PolygonShape m_polygonA;
		PolygonShape m_polygonB;
	};
}
