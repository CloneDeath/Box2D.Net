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
	/// This tests stacking. It also shows how to use World::Query
	/// and TestOverlap.

	/// This callback is called by World::QueryAABB. We find all the fixtures
	/// that overlap an AABB. Of those, we use TestOverlap to determine which fixtures
	/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
	class PolyShapesCallback : QueryCallback
	{	
		const int e_maxCount = 4;

		public PolyShapesCallback()
		{
			m_count = 0;
		}

		public void DrawFixture(Fixture fixture)
		{
			Color color = Color.FromArgb(245, 245, 150);
			Transform xf = fixture.GetBody().GetTransform();

			switch (fixture.GetShapeType())
			{
			case ShapeType.Circle:
				{
					CircleShape circle = (CircleShape)fixture.GetShape();

					Vec2 center = Utilities.Mul(xf, circle.m_p);
					float radius = circle.m_radius;

					m_debugDraw.DrawCircle(center, radius, color);
				}
				break;

			case ShapeType.Polygon:
				{
					PolygonShape poly = (PolygonShape)fixture.GetShape();
					int vertexCount = poly.m_count;
					Utilities.Assert(vertexCount <= Settings._maxPolygonVertices);
					Vec2[] vertices = new Vec2[Settings._maxPolygonVertices];

					for (int i = 0; i < vertexCount; ++i)
					{
						vertices[i] = Utilities.Mul(xf, poly.m_vertices[i]);
					}

					m_debugDraw.DrawPolygon(vertices, vertexCount, color);
				}
				break;
				
			default:
				break;
			}
		}

		/// Called for each fixture found in the query AABB.
		/// @return false to terminate the query.
		public override bool ReportFixture(Fixture fixture)
		{
			if (m_count == e_maxCount)
			{
				return false;
			}

			Body body = fixture.GetBody();
			Shape shape = fixture.GetShape();

			bool overlap = Collision.TestOverlap(shape, 0, m_circle, 0, body.GetTransform(), m_transform);

			if (overlap)
			{
				DrawFixture(fixture);
				++m_count;
			}

			return true;
		}

		public CircleShape m_circle;
		public Transform m_transform;
		public Draw m_debugDraw;
		public int m_count;
	};

	class PolyShapes : Test
	{
		const int k_maxBodies = 256;
		public PolyShapes()
		{
			// Ground body
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				Vec2[] vertices = new Vec2[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);
				m_polygons[0].Set(vertices, 3);
			}
		
			{
				Vec2[] vertices = new Vec2[3];
				vertices[0].Set(-0.1f, 0.0f);
				vertices[1].Set(0.1f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);
				m_polygons[1].Set(vertices, 3);
			}

			{
				float w = 1.0f;
				float b = w / (2.0f + (float)Math.Sqrt(2.0f));
				float s = (float)Math.Sqrt(2.0f) * b;

				Vec2[] vertices = new Vec2[8];
				vertices[0].Set(0.5f * s, 0.0f);
				vertices[1].Set(0.5f * w, b);
				vertices[2].Set(0.5f * w, b + s);
				vertices[3].Set(0.5f * s, w);
				vertices[4].Set(-0.5f * s, w);
				vertices[5].Set(-0.5f * w, b + s);
				vertices[6].Set(-0.5f * w, b);
				vertices[7].Set(-0.5f * s, 0.0f);

				m_polygons[2].Set(vertices, 8);
			}

			{
				m_polygons[3].SetAsBox(0.5f, 0.5f);
			}

			{
				m_circle.m_radius = 0.5f;
			}

			m_bodyIndex = 0;
			Array.Clear(m_bodies, 0, m_bodies.Length);
		}

		public void Create(int index)
		{
			if (m_bodies[m_bodyIndex] != null)
			{
				m_world.DestroyBody(m_bodies[m_bodyIndex]);
				m_bodies[m_bodyIndex] = null;
			}

			BodyDef bd = new BodyDef();
			bd.type = BodyType._dynamicBody;

			float x = RandomFloat(-2.0f, 2.0f);
			bd.position.Set(x, 10.0f);
			bd.angle = RandomFloat(-(float)Math.PI, (float)Math.PI);

			if (index == 4)
			{
				bd.angularDamping = 0.02f;
			}

			m_bodies[m_bodyIndex] = m_world.CreateBody(bd);

			if (index < 4)
			{
				FixtureDef fd = new FixtureDef();
				fd.shape = m_polygons[index];
				fd.density = 1.0f;
				fd.friction = 0.3f;
				m_bodies[m_bodyIndex].CreateFixture(fd);
			}
			else
			{
				FixtureDef fd = new FixtureDef();
				fd.shape = m_circle;
				fd.density = 1.0f;
				fd.friction = 0.3f;

				m_bodies[m_bodyIndex].CreateFixture(fd);
			}

			m_bodyIndex = (m_bodyIndex + 1) % k_maxBodies;
		}

		public void DestroyBody()
		{
			for (int i = 0; i < k_maxBodies; ++i)
			{
				if (m_bodies[i] != null)
				{
					m_world.DestroyBody(m_bodies[i]);
					m_bodies[i] = null;
					return;
				}
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.Number1)){
				Create(0);
			}
			if (KeyboardManager.IsPressed(Key.Number2)) {
				Create(1);
			}
			if (KeyboardManager.IsPressed(Key.Number3)) {
				Create(2);
			}
			if (KeyboardManager.IsPressed(Key.Number4)) {
				Create(3);
			}
			if (KeyboardManager.IsPressed(Key.Number5)) {
				Create(4);
			}

			if (KeyboardManager.IsPressed(Key.A)) {
				for (int i = 0; i < k_maxBodies; i += 2) {
					if (m_bodies[i] != null) {
						bool active = m_bodies[i].IsActive();
						m_bodies[i].SetActive(!active);
					}
				}
			}

			if (KeyboardManager.IsPressed(Key.D)){
				DestroyBody();
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			PolyShapesCallback callback = new PolyShapesCallback();
			callback.m_circle.m_radius = 2.0f;
			callback.m_circle.m_p.Set(0.0f, 1.1f);
			callback.m_transform.SetIdentity();
			callback.m_debugDraw = m_debugDraw;

			AABB aabb;
			callback.m_circle.ComputeAABB(out aabb, callback.m_transform, 0);

			m_world.QueryAABB(callback, aabb);

			Color color = Color.FromArgb(100, 175, 200);
			m_debugDraw.DrawCircle(callback.m_circle.m_p, callback.m_circle.m_radius, color);

			m_debugDraw.DrawString("Press 1-5 to drop stuff");
			
			m_debugDraw.DrawString("Press 'a' to (de)activate some bodies");
			
			m_debugDraw.DrawString("Press 'd' to destroy a body");
			
		}

		public static Test Create()
		{
			return new PolyShapes();
		}

		int m_bodyIndex;
		Body[] m_bodies = new Body[k_maxBodies];
		PolygonShape[] m_polygons = new PolygonShape[4];
		CircleShape m_circle;
	};
}
