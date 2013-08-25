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
	class PolyCollision : Test
	{
		public PolyCollision()
		{
			{
				m_polygonA = new PolygonShape();
				m_polygonA.SetAsBox(0.2f, 0.4f);
				m_transformA.Set(new Vec2(0.0f, 0.0f), 0.0f);
			}

			{
				m_polygonB = new PolygonShape();
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

		public override void Step(TestSettings settings)
		{
			Manifold manifold;
			Collision.CollidePolygons(out manifold, m_polygonA, m_transformA, m_polygonB, m_transformB);

			WorldManifold worldManifold = new WorldManifold();
			worldManifold.Initialize(manifold, m_transformA, m_polygonA.m_radius, m_transformB, m_polygonB.m_radius);

			m_debugDraw.DrawString("point count = {0}", manifold.points.Count());
			

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

			for (int i = 0; i < manifold.points.Count(); ++i)
			{
				m_debugDraw.DrawPoint(worldManifold.points[i], 4.0f, Color.FromArgb(225, 75, 75));
			}
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

			if (KeyboardManager.IsPressed(Key.E)){
				m_angleB -= 0.1f * (float)Math.PI;
			}

			m_transformB.Set(m_positionB, m_angleB);
		}

		PolygonShape m_polygonA;
		PolygonShape m_polygonB;

		Transform m_transformA;
		Transform m_transformB;

		Vec2 m_positionB;
		float m_angleB;
	};
}
