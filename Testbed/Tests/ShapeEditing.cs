using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class ShapeEditing : Test
	{
		public ShapeEditing()
		{
			{
				BodyDef bd1 = new BodyDef();
				Body ground = m_world.CreateBody(bd1);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			BodyDef bd = new BodyDef();
			bd.type = BodyType._dynamicBody;
			bd.position.Set(0.0f, 10.0f);
			m_body = m_world.CreateBody(bd);

			PolygonShape shape2 = new PolygonShape();
			shape2.SetAsBox(4.0f, 4.0f, new Vec2(0.0f, 0.0f), 0.0f);
			m_fixture1 = m_body.CreateFixture(shape2, 10.0f);

			m_fixture2 = null;

			m_sensor = false;
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.C)) {
				if (m_fixture2 == null) {
					CircleShape shape = new CircleShape();
					shape.m_radius = 3.0f;
					shape.m_p.Set(0.5f, -4.0f);
					m_fixture2 = m_body.CreateFixture(shape, 10.0f);
					m_body.SetAwake(true);
				}
			}

			if (KeyboardManager.IsPressed(Key.D)){
				if (m_fixture2 != null)
				{
					m_body.DestroyFixture(m_fixture2);
					m_fixture2 = null;
					m_body.SetAwake(true);
				}
			}

			if (KeyboardManager.IsPressed(Key.S)){
				if (m_fixture2 != null)
				{
					m_sensor = !m_sensor;
					m_fixture2.SetSensor(m_sensor);
				}
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Press: (c) create a shape, (d) destroy a shape.");
			
			m_debugDraw.DrawString("sensor = %d", m_sensor);
			
		}

		public static Test Create()
		{
			return new ShapeEditing();
		}

		Body m_body;
		Fixture m_fixture1;
		Fixture m_fixture2;
		bool m_sensor;
	};
}
