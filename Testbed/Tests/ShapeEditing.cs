using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class ShapeEditing : Test
	{
		public ShapeEditing()
		{
			{
				b2BodyDef bd1 = new b2BodyDef();
				b2Body ground = m_world.CreateBody(bd1);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			b2BodyDef bd = new b2BodyDef();
			bd.type = b2BodyType.b2_dynamicBody;
			bd.position.Set(0.0f, 10.0f);
			m_body = m_world.CreateBody(bd);

			b2PolygonShape shape2 = new b2PolygonShape();
			shape2.SetAsBox(4.0f, 4.0f, new b2Vec2(0.0f, 0.0f), 0.0f);
			m_fixture1 = m_body.CreateFixture(shape2, 10.0f);

			m_fixture2 = null;

			m_sensor = false;
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 'c':
				if (m_fixture2 == null)
				{
					b2CircleShape shape = new b2CircleShape();
					shape.m_radius = 3.0f;
					shape.m_p.Set(0.5f, -4.0f);
					m_fixture2 = m_body.CreateFixture(shape, 10.0f);
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

		public override void Step(Settings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Press: (c) create a shape, (d) destroy a shape.");
			
			m_debugDraw.DrawString("sensor = %d", m_sensor);
			
		}

		public static Test Create()
		{
			return new ShapeEditing();
		}

		b2Body m_body;
		b2Fixture m_fixture1;
		b2Fixture m_fixture2;
		bool m_sensor;
	};
}
