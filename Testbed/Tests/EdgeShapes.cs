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
	class EdgeShapesCallback : RayCastCallback
	{
		public EdgeShapesCallback()
		{
			m_fixture = null;
		}

		public override float ReportFixture(Fixture fixture, Vec2 point,
			Vec2 normal, float fraction)
		{
			m_fixture = fixture;
			m_point = point;
			m_normal = normal;

			return fraction;
		}

		public Fixture m_fixture;
		public Vec2 m_point;
		public Vec2 m_normal;
	};

	class EdgeShapes : Test
	{
		const int e_maxBodies = 256;

		public EdgeShapes()
		{
			// Ground body
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				float x1 = -20.0f;
				float y1 = 2.0f * (float)Math.Cos(x1 / 10.0f * (float)Math.PI);
				for (int i = 0; i < 80; ++i)
				{
					float x2 = x1 + 0.5f;
					float y2 = 2.0f * (float)Math.Cos(x2 / 10.0f * (float)Math.PI);

					EdgeShape shape = new EdgeShape();
					shape.Set(new Vec2(x1, y1), new Vec2(x2, y2));
					ground.CreateFixture(shape, 0.0f);

					x1 = x2;
					y1 = y2;
				}
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

			m_angle = 0.0f;
		}

		public void Create(int index)
		{
			if (m_bodies[m_bodyIndex] != null)
			{
				m_world.DestroyBody(m_bodies[m_bodyIndex]);
				m_bodies[m_bodyIndex] = null;
			}

			BodyDef bd = new BodyDef();

			float x = RandomFloat(-10.0f, 10.0f);
			float y = RandomFloat(10.0f, 20.0f);
			bd.position.Set(x, y);
			bd.angle = RandomFloat(-(float)Math.PI, (float)Math.PI);
			bd.type = BodyType._dynamicBody;

			if (index == 4)
			{
				bd.angularDamping = 0.02f;
			}

			m_bodies[m_bodyIndex] = m_world.CreateBody(bd);

			if (index < 4)
			{
				FixtureDef fd = new FixtureDef();
				fd.shape = m_polygons[index];
				fd.friction = 0.3f;
				fd.density = 20.0f;
				m_bodies[m_bodyIndex].CreateFixture(fd);
			}
			else
			{
				FixtureDef fd = new FixtureDef();
				fd.shape = m_circle;
				fd.friction = 0.3f;
				fd.density = 20.0f;
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

			if (KeyboardManager.IsPressed(Key.D)){
				DestroyBody();
			}
		}

		public override void Step(TestSettings settings)
		{
			bool advanceRay = settings.pause == false || settings.singleStep;

			base.Step(settings);
			m_debugDraw.DrawString("Press 1-5 to drop stuff");
			

			float L = 25.0f;
			Vec2 point1 = new Vec2(0.0f, 10.0f);
			Vec2 d = new Vec2(L * (float)Math.Cos(m_angle), -L * Math.Abs((float)Math.Sin(m_angle)));
			Vec2 point2 = point1 + d;

			EdgeShapesCallback callback = new EdgeShapesCallback();

			m_world.RayCast(callback, point1, point2);

			if (callback.m_fixture != null)
			{
				m_debugDraw.DrawPoint(callback.m_point, 5.0f, Color.FromArgb(100, 225, 100));

				m_debugDraw.DrawSegment(point1, callback.m_point, Color.FromArgb(200, 200, 200));

				Vec2 head = callback.m_point + 0.5f * callback.m_normal;
				m_debugDraw.DrawSegment(callback.m_point, head, Color.FromArgb(225, 225, 100));
			}
			else
			{
				m_debugDraw.DrawSegment(point1, point2, Color.FromArgb(200, 200, 200));
			}

			if (advanceRay)
			{
				m_angle += 0.25f * (float)Math.PI / 180.0f;
			}
		}

		public static Test Create()
		{
			return new EdgeShapes();
		}

		int m_bodyIndex;
		Body[] m_bodies = new Body[e_maxBodies];
		PolygonShape[] m_polygons = new PolygonShape[4];
		CircleShape m_circle;

		float m_angle;
	};
}
