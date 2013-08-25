﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;

namespace Testbed.Tests {
	// This test demonstrates how to use the world ray-cast feature.
	// NOTE: we are intentionally filtering one of the polygons, therefore
	// the ray will always miss one type of polygon.

	// This callback finds the closest hit. Polygon 0 is filtered.
	class RayCastClosestCallback : b2RayCastCallback
	{
		public RayCastClosestCallback()
		{
			m_hit = false;
		}

		public override float ReportFixture(b2Fixture fixture, b2Vec2 point, b2Vec2 normal, float fraction)
		{
			b2Body body = fixture.GetBody();
			object userData = body.GetUserData();
			if (userData != null)
			{
				int index = (int)userData;
				if (index == 0)
				{
					// By returning -1, we instruct the calling code to ignore this fixture and
					// continue the ray-cast to the next fixture.
					return -1.0f;
				}
			}

			m_hit = true;
			m_point = point;
			m_normal = normal;

			// By returning the current fraction, we instruct the calling code to clip the ray and
			// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
			// are reported in order. However, by clipping, we can always get the closest fixture.
			return fraction;
		}

		public bool m_hit;
		public b2Vec2 m_point;
		public b2Vec2 m_normal;
	}

	// This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
	// just checking for obstruction, so the actual fixture and hit point are irrelevant. 
	class RayCastAnyCallback : b2RayCastCallback
	{
		public RayCastAnyCallback()
		{
			m_hit = false;
		}

		public override float ReportFixture(b2Fixture fixture, b2Vec2 point, b2Vec2 normal, float fraction)
		{
			b2Body body = fixture.GetBody();
			object userData = body.GetUserData();
			if (userData != null)
			{
				int index = (int)userData;
				if (index == 0)
				{
					// By returning -1, we instruct the calling code to ignore this fixture
					// and continue the ray-cast to the next fixture.
					return -1.0f;
				}
			}

			m_hit = true;
			m_point = point;
			m_normal = normal;

			// At this point we have a hit, so we know the ray is obstructed.
			// By returning 0, we instruct the calling code to terminate the ray-cast.
			return 0.0f;
		}

		public bool m_hit;
		public b2Vec2 m_point;
		public b2Vec2 m_normal;
	};

	// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
	// The fixtures are not necessary reported in order, so we might not capture
	// the closest fixture.
	class RayCastMultipleCallback : b2RayCastCallback
	{
		const int e_maxCount = 3;

		public RayCastMultipleCallback()
		{
			m_count = 0;
		}

		public override float ReportFixture(b2Fixture fixture, b2Vec2 point, b2Vec2 normal, float fraction)
		{
			b2Body body = fixture.GetBody();
			object userData = body.GetUserData();
			if (userData != null)
			{
				int index = (int)userData;
				if (index == 0)
				{
					// By returning -1, we instruct the calling code to ignore this fixture
					// and continue the ray-cast to the next fixture.
					return -1.0f;
				}
			}

			Utilities.Assert(m_count < e_maxCount);

			m_points[m_count] = point;
			m_normals[m_count] = normal;
			++m_count;

			if (m_count == e_maxCount)
			{
				// At this point the buffer is full.
				// By returning 0, we instruct the calling code to terminate the ray-cast.
				return 0.0f;
			}

			// By returning 1, we instruct the caller to continue without clipping the ray.
			return 1.0f;
		}

		public b2Vec2[] m_points = new b2Vec2[e_maxCount];
		public b2Vec2[] m_normals = new b2Vec2[e_maxCount];
		public int m_count;
	};


	class RayCast : Test
	{
		const int e_maxBodies = 256;

		public enum Mode
		{
			e_closest,
			e_any,
			e_multiple
		};

		public RayCast()
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

			m_angle = 0.0f;

			m_mode = Mode.e_closest;
		}

		public void Create(int index)
		{
			if (m_bodies[m_bodyIndex] != null)
			{
				m_world.DestroyBody(m_bodies[m_bodyIndex]);
				m_bodies[m_bodyIndex] = null;
			}

			b2BodyDef bd = new b2BodyDef();

			float x = RandomFloat(-10.0f, 10.0f);
			float y = RandomFloat(0.0f, 20.0f);
			bd.position.Set(x, y);
			bd.angle = RandomFloat(-(float)Math.PI, (float)Math.PI);

			m_userData[m_bodyIndex] = index;
			bd.userData = m_userData[m_bodyIndex];

			if (index == 4)
			{
				bd.angularDamping = 0.02f;
			}

			m_bodies[m_bodyIndex] = m_world.CreateBody(bd);

			if (index < 4)
			{
				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = m_polygons[index];
				fd.friction = 0.3f;
				m_bodies[m_bodyIndex].CreateFixture(fd);
			}
			else
			{
				b2FixtureDef fd = new b2FixtureDef();
				fd.shape = m_circle;
				fd.friction = 0.3f;

				m_bodies[m_bodyIndex].CreateFixture(fd);
			}

			m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
		}

		public void DestroyBody()
		{
			for (int i = 0; i < e_maxBodies; ++i)
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

			case 'd':
				DestroyBody();
				break;

			case 'm':
				if (m_mode == e_closest)
				{
					m_mode = e_any;
				}
				else if (m_mode == e_any)
				{
					m_mode = e_multiple;
				}
				else if (m_mode == e_multiple)
				{
					m_mode = e_closest;
				}
			}
		}

		public override void Step(Settings settings)
		{
			bool advanceRay = settings.pause == false || settings.singleStep;

			base.Step(settings);
			m_debugDraw.DrawString("Press 1-5 to drop stuff, m to change the mode");
			
			switch (m_mode)
			{
				case Mode.e_closest:
					m_debugDraw.DrawString("Ray-cast mode: closest - find closest fixture along the ray");
					break;

				case Mode.e_any:
					m_debugDraw.DrawString("Ray-cast mode: any - check for obstruction");
					break;

				case Mode.e_multiple:
					m_debugDraw.DrawString("Ray-cast mode: multiple - gather multiple fixtures");
					break;
			}

			float L = 11.0f;
			b2Vec2 point1 = new b2Vec2(0.0f, 10.0f);
			b2Vec2 d = new b2Vec2(L * (float)Math.Cos(m_angle), L * (float)Math.Sin(m_angle));
			b2Vec2 point2 = point1 + d;

			if (m_mode == Mode.e_closest)
			{
				RayCastClosestCallback callback = new RayCastClosestCallback();
				m_world.RayCast(callback, point1, point2);

				if (callback.m_hit)
				{
					m_debugDraw.DrawPoint(callback.m_point, 5.0f, Color.FromArgb(100, 225, 100));
					m_debugDraw.DrawSegment(point1, callback.m_point, Color.FromArgb(200, 200, 200));
					b2Vec2 head = callback.m_point + 0.5f * callback.m_normal;
					m_debugDraw.DrawSegment(callback.m_point, head, Color.FromArgb(225, 225, 100));
				}
				else
				{
					m_debugDraw.DrawSegment(point1, point2, Color.FromArgb(200, 200, 200));
				}
			}
			else if (m_mode == Mode.e_any)
			{
				RayCastAnyCallback callback = new RayCastAnyCallback();
				m_world.RayCast(callback, point1, point2);

				if (callback.m_hit)
				{
					m_debugDraw.DrawPoint(callback.m_point, 5.0f, Color.FromArgb(100, 225, 100));
					m_debugDraw.DrawSegment(point1, callback.m_point, Color.FromArgb(200, 200, 200));
					b2Vec2 head = callback.m_point + 0.5f * callback.m_normal;
					m_debugDraw.DrawSegment(callback.m_point, head, Color.FromArgb(225, 225, 100));
				}
				else
				{
					m_debugDraw.DrawSegment(point1, point2, Color.FromArgb(200, 200, 200));
				}
			}
			else if (m_mode == Mode.e_multiple)
			{
				RayCastMultipleCallback callback = new RayCastMultipleCallback();
				m_world.RayCast(callback, point1, point2);
				m_debugDraw.DrawSegment(point1, point2, Color.FromArgb(200, 200, 200));

				for (int i = 0; i < callback.m_count; ++i)
				{
					b2Vec2 p = callback.m_points[i];
					b2Vec2 n = callback.m_normals[i];
					m_debugDraw.DrawPoint(p, 5.0f, Color.FromArgb(100, 225, 100));
					m_debugDraw.DrawSegment(point1, p, Color.FromArgb(200, 200, 200));
					b2Vec2 head = p + 0.5f * n;
					m_debugDraw.DrawSegment(p, head, Color.FromArgb(225, 225, 100));
				}
			}

			if (advanceRay)
			{
				m_angle += 0.25f * (float)Math.PI / 180.0f;
			}

	#if ZERO
			// This case was failing.
			{
				b2Vec2[] vertices = new b2Vec2[4];
				//vertices[0].Set(-22.875f, -3.0f);
				//vertices[1].Set(22.875f, -3.0f);
				//vertices[2].Set(22.875f, 3.0f);
				//vertices[3].Set(-22.875f, 3.0f);

				b2PolygonShape shape = new b2PolygonShape();
				//shape.Set(vertices, 4);
				shape.SetAsBox(22.875f, 3.0f);

				b2RayCastInput input;
				input.p1.Set(10.2725f,1.71372f);
				input.p2.Set(10.2353f,2.21807f);
				//input.maxFraction = 0.567623f;
				input.maxFraction = 0.56762173f;

				b2Transform xf;
				xf.SetIdentity();
				xf.position.Set(23.0f, 5.0f);

				b2RayCastOutput output;
				bool hit;
				hit = shape.RayCast(out output, input, xf);
				hit = false;

				Color color(1.0f, 1.0f, 1.0f);
				b2Vec2[] vs = new b2Vec2[4];
				for (int i = 0; i < 4; ++i)
				{
					vs[i] = Utilities.b2Mul(xf, shape.m_vertices[i]);
				}

				m_debugDraw.DrawPolygon(vs, 4, color);
				m_debugDraw.DrawSegment(input.p1, input.p2, color);
			}
	#endif
		}

		public static Test Create()
		{
			return new RayCast();
		}

		int m_bodyIndex;
		b2Body[] m_bodies = new b2Body[e_maxBodies];
		int[] m_userData = new int[e_maxBodies];
		b2PolygonShape[] m_polygons = new b2PolygonShape[4];
		b2CircleShape m_circle;

		float m_angle;

		Mode m_mode;
	};
}
