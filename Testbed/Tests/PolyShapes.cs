using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;
using System.Drawing;

namespace Testbed.Tests {
	/// This tests stacking. It also shows how to use b2World::Query
	/// and b2TestOverlap.

	/// This callback is called by b2World::QueryAABB. We find all the fixtures
	/// that overlap an AABB. Of those, we use b2TestOverlap to determine which fixtures
	/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
	class PolyShapesCallback : b2QueryCallback
	{	
		const int e_maxCount = 4;

		public PolyShapesCallback()
		{
			m_count = 0;
		}

		public void DrawFixture(b2Fixture fixture)
		{
			Color color = Color.FromArgb(245, 245, 150);
			b2Transform xf = fixture.GetBody().GetTransform();

			switch (fixture.GetShapeType())
			{
			case ShapeType.Circle:
				{
					b2CircleShape circle = (b2CircleShape)fixture.GetShape();

					b2Vec2 center = Utilities.b2Mul(xf, circle.m_p);
					float radius = circle.m_radius;

					m_debugDraw.DrawCircle(center, radius, color);
				}
				break;

			case ShapeType.Polygon:
				{
					b2PolygonShape poly = (b2PolygonShape)fixture.GetShape();
					int vertexCount = poly.m_count;
					Utilities.Assert(vertexCount <= b2Settings.b2_maxPolygonVertices);
					b2Vec2[] vertices = new b2Vec2[b2Settings.b2_maxPolygonVertices];

					for (int i = 0; i < vertexCount; ++i)
					{
						vertices[i] = Utilities.b2Mul(xf, poly.m_vertices[i]);
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
		public override bool ReportFixture(b2Fixture fixture)
		{
			if (m_count == e_maxCount)
			{
				return false;
			}

			b2Body body = fixture.GetBody();
			b2Shape shape = fixture.GetShape();

			bool overlap = b2Collision.b2TestOverlap(shape, 0, m_circle, 0, body.GetTransform(), m_transform);

			if (overlap)
			{
				DrawFixture(fixture);
				++m_count;
			}

			return true;
		}

		public b2CircleShape m_circle;
		public b2Transform m_transform;
		public b2Draw m_debugDraw;
		public int m_count;
	};

	class PolyShapes : Test
	{
		const int k_maxBodies = 256;
		public PolyShapes()
		{
			// Ground body
			{
				b2BodyDef bd = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				b2Vec2[] vertices = new b2Vec2[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);
				m_polygons[0].Set(vertices, 3);
			}
		
			{
				b2Vec2[] vertices = new b2Vec2[3];
				vertices[0].Set(-0.1f, 0.0f);
				vertices[1].Set(0.1f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);
				m_polygons[1].Set(vertices, 3);
			}

			{
				float w = 1.0f;
				float b = w / (2.0f + (float)Math.Sqrt(2.0f));
				float s = (float)Math.Sqrt(2.0f) * b;

				b2Vec2[] vertices = new b2Vec2[8];
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

			b2BodyDef bd = new b2BodyDef();
			bd.type = b2BodyType.b2_dynamicBody;

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
				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = m_polygons[index];
				fd.density = 1.0f;
				fd.friction = 0.3f;
				m_bodies[m_bodyIndex].CreateFixture(fd);
			}
			else
			{
				b2FixtureDef fd = new b2FixtureDef();
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
			switch (key)
			{
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
				Create(key - '1');
				break;

			case 'a':
				for (int i = 0; i < k_maxBodies; i += 2)
				{
					if (m_bodies[i])
					{
						bool active = m_bodies[i].IsActive();
						m_bodies[i].SetActive(!active);
					}
				}
				break;

			case 'd':
				DestroyBody();
				break;
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			PolyShapesCallback callback = new PolyShapesCallback();
			callback.m_circle.m_radius = 2.0f;
			callback.m_circle.m_p.Set(0.0f, 1.1f);
			callback.m_transform.SetIdentity();
			callback.m_debugDraw = m_debugDraw;

			b2AABB aabb;
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
		b2Body[] m_bodies = new b2Body[k_maxBodies];
		b2PolygonShape[] m_polygons = new b2PolygonShape[4];
		b2CircleShape m_circle;
	};
}
