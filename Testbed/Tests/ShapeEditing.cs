using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class ShapeEditing : Test
	{
		public ShapeEditing()
		{
			{
				b2BodyDef bd;
				b2Body* ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 10.0f);
			m_body = m_world.CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(4.0f, 4.0f, b2Vec2(0.0f, 0.0f), 0.0f);
			m_fixture1 = m_body.CreateFixture(&shape, 10.0f);

			m_fixture2 = null;

			m_sensor = false;
		}

		public void Keyboard(unsigned char key)
		{
			switch (key)
			{
			case 'c':
				if (m_fixture2 == null)
				{
					b2CircleShape shape;
					shape.m_radius = 3.0f;
					shape.m_p.Set(0.5f, -4.0f);
					m_fixture2 = m_body.CreateFixture(&shape, 10.0f);
					m_body.SetAwake(true);
				}
				break;

			case 'd':
				if (m_fixture2 != null)
				{
					m_body.DestroyFixture(m_fixture2);
					m_fixture2 = null;
					m_body.SetAwake(true);
				}
				break;

			case 's':
				if (m_fixture2 != null)
				{
					m_sensor = !m_sensor;
					m_fixture2.SetSensor(m_sensor);
				}
				break;
			}
		}

		public void Step(Settings* settings)
		{
			Test::Step(settings);
			m_debugDraw.DrawString(5, m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
			m_textLine += DRAW_STRING_NEW_LINE;
			m_debugDraw.DrawString(5, m_textLine, "sensor = %d", m_sensor);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		public static Test Create()
		{
			return new ShapeEditing();
		}

		b2Body* m_body;
		b2Fixture* m_fixture1;
		b2Fixture* m_fixture2;
		bool m_sensor;
	};
}
