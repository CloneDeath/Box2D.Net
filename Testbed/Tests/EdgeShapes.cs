﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class EdgeShapesCallback : public b2RayCastCallback
	{
	public:
		EdgeShapesCallback()
		{
			m_fixture = null;
		}

		float ReportFixture(	b2Fixture* fixture, b2Vec2 point,
			b2Vec2 normal, float fraction)
		{
			m_fixture = fixture;
			m_point = point;
			m_normal = normal;

			return fraction;
		}

		b2Fixture* m_fixture;
		b2Vec2 m_point;
		b2Vec2 m_normal;
	};

	class EdgeShapes : Test
	{
	public:

		enum
		{
			e_maxBodies = 256
		};

		EdgeShapes()
		{
			// Ground body
			{
				b2BodyDef bd;
				b2Body* ground = m_world.CreateBody(&bd);

				float x1 = -20.0f;
				float y1 = 2.0f * cosf(x1 / 10.0f * Math.PI);
				for (int i = 0; i < 80; ++i)
				{
					float x2 = x1 + 0.5f;
					float y2 = 2.0f * cosf(x2 / 10.0f * Math.PI);

					b2EdgeShape shape;
					shape.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
					ground.CreateFixture(&shape, 0.0f);

					x1 = x2;
					y1 = y2;
				}
			}

			{
				b2Vec2 vertices[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);
				m_polygons[0].Set(vertices, 3);
			}

			{
				b2Vec2 vertices[3];
				vertices[0].Set(-0.1f, 0.0f);
				vertices[1].Set(0.1f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);
				m_polygons[1].Set(vertices, 3);
			}

			{
				float w = 1.0f;
				float b = w / (2.0f + b2Sqrt(2.0f));
				float s = b2Sqrt(2.0f) * b;

				b2Vec2 vertices[8];
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
			memset(m_bodies, 0, sizeof(m_bodies));

			m_angle = 0.0f;
		}

		void Create(int index)
		{
			if (m_bodies[m_bodyIndex] != null)
			{
				m_world.DestroyBody(m_bodies[m_bodyIndex]);
				m_bodies[m_bodyIndex] = null;
			}

			b2BodyDef bd;

			float x = RandomFloat(-10.0f, 10.0f);
			float y = RandomFloat(10.0f, 20.0f);
			bd.position.Set(x, y);
			bd.angle = RandomFloat(-Math.PI, Math.PI);
			bd.type = b2_dynamicBody;

			if (index == 4)
			{
				bd.angularDamping = 0.02f;
			}

			m_bodies[m_bodyIndex] = m_world.CreateBody(&bd);

			if (index < 4)
			{
				b2FixtureDef fd;
				fd.shape = m_polygons + index;
				fd.friction = 0.3f;
				fd.density = 20.0f;
				m_bodies[m_bodyIndex].CreateFixture(&fd);
			}
			else
			{
				b2FixtureDef fd;
				fd.shape = &m_circle;
				fd.friction = 0.3f;
				fd.density = 20.0f;
				m_bodies[m_bodyIndex].CreateFixture(&fd);
			}

			m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
		}

		void DestroyBody()
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

		void Keyboard(unsigned char key)
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
			}
		}

		void Step(Settings* settings)
		{
			bool advanceRay = settings.pause == 0 || settings.singleStep;

			Test::Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Press 1-5 to drop stuff");
			m_textLine += DRAW_STRING_NEW_LINE;

			float L = 25.0f;
			b2Vec2 point1(0.0f, 10.0f);
			b2Vec2 d(L * cosf(m_angle), -L * Math.Abs(sinf(m_angle)));
			b2Vec2 point2 = point1 + d;

			EdgeShapesCallback callback;

			m_world.RayCast(&callback, point1, point2);

			if (callback.m_fixture)
			{
				m_debugDraw.DrawPoint(callback.m_point, 5.0f, b2Color(0.4f, 0.9f, 0.4f));

				m_debugDraw.DrawSegment(point1, callback.m_point, b2Color(0.8f, 0.8f, 0.8f));

				b2Vec2 head = callback.m_point + 0.5f * callback.m_normal;
				m_debugDraw.DrawSegment(callback.m_point, head, b2Color(0.9f, 0.9f, 0.4f));
			}
			else
			{
				m_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));
			}

			if (advanceRay)
			{
				m_angle += 0.25f * Math.PI / 180.0f;
			}
		}

		static Test* Create()
		{
			return new EdgeShapes;
		}

		int m_bodyIndex;
		b2Body* m_bodies[e_maxBodies];
		b2PolygonShape m_polygons[4];
		b2CircleShape m_circle;

		float m_angle;
	};
}
