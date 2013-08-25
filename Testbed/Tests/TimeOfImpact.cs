using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;

namespace Testbed.Tests {
	class TimeOfImpact : Test
	{
		PolygonShape m_shapeA;
		PolygonShape m_shapeB;

		public TimeOfImpact()
		{
			m_shapeA = new PolygonShape();
			m_shapeB = new PolygonShape();
			m_shapeA.SetAsBox(25.0f, 5.0f);
			m_shapeB.SetAsBox(2.5f, 2.5f);
		}

		public static Test Create()
		{
			return new TimeOfImpact();
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			Sweep sweepA = new Sweep();
			sweepA.c0.Set(24.0f, -60.0f);
			sweepA.a0 = 2.95f;
			sweepA.c = sweepA.c0;
			sweepA.a = sweepA.a0;
			sweepA.localCenter.SetZero();

			Sweep sweepB = new Sweep();
			sweepB.c0.Set(53.474274f, -50.252514f);
			sweepB.a0 = 513.36676f; // - 162.0f * (float)Math.PI;
			sweepB.c.Set(54.595478f, -51.083473f);
			sweepB.a = 513.62781f; //  - 162.0f * (float)Math.PI;
			sweepB.localCenter.SetZero();

			//sweepB.a0 -= 300.0f * (float)Math.PI;
			//sweepB.a -= 300.0f * (float)Math.PI;

			TOIInput input = new TOIInput();
			input.proxyA.Set(m_shapeA, 0);
			input.proxyB.Set(m_shapeB, 0);
			input.sweepA = sweepA;
			input.sweepB = sweepB;
			input.tMax = 1.0f;

			TOIOutput output;

			Utilities.TimeOfImpact(out output, input);

			m_debugDraw.DrawString("toi = {0}", output.t);

			m_debugDraw.DrawString("max toi iters = {0}, max root iters = {1}", Utilities._toiMaxIters, Utilities._toiMaxRootIters);

			Vec2[] vertices = new Vec2[Settings._maxPolygonVertices];

			Transform transformA;
			sweepA.GetTransform(out transformA, 0.0f);
			for (int i = 0; i < m_shapeA.m_count; ++i)
			{
				vertices[i] = Utilities.Mul(transformA, m_shapeA.m_vertices[i]);
			}
			m_debugDraw.DrawPolygon(vertices, m_shapeA.m_count, Color.FromArgb(225, 225, 225));

			Transform transformB;
			sweepB.GetTransform(out transformB, 0.0f);
		
			//Vec2 localPoint(2.0f, -0.1f);

			for (int i = 0; i < m_shapeB.m_count; ++i)
			{
				vertices[i] = Utilities.Mul(transformB, m_shapeB.m_vertices[i]);
			}
			m_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, Color.FromArgb(128, 225, 128));

			sweepB.GetTransform(out transformB, output.t);
			for (int i = 0; i < m_shapeB.m_count; ++i)
			{
				vertices[i] = Utilities.Mul(transformB, m_shapeB.m_vertices[i]);
			}
			m_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, Color.FromArgb(128, 175, 225));

			sweepB.GetTransform(out transformB, 1.0f);
			for (int i = 0; i < m_shapeB.m_count; ++i)
			{
				vertices[i] = Utilities.Mul(transformB, m_shapeB.m_vertices[i]);
			}
			m_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, Color.FromArgb(225, 128, 128));

	#if ZERO
			for (float t = 0.0f; t < 1.0f; t += 0.1f)
			{
				sweepB.GetTransform(out transformB, t);
				for (int i = 0; i < m_shapeB.m_count; ++i)
				{
					vertices[i] = Utilities.Mul(transformB, m_shapeB.m_vertices[i]);
				}
				m_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, Color.FromArgb(225, 0.5f, 0.5f));
			}
	#endif
		}
	}
}
