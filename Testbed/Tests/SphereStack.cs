using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class SphereStack : Test
	{
		const int e_count = 10;

		public SphereStack()
		{
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 1.0f;

				for (int i = 0; i < e_count; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.Position.Set(0.0f, 4.0f + 3.0f * i);

					m_bodies[i] = m_world.CreateBody(bd);

					m_bodies[i].CreateFixture(shape);

					m_bodies[i].SetLinearVelocity(new Vec2(0.0f, -50.0f));
				}
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);

			//for (int i = 0; i < e_count; ++i)
			//{
			//	printf("%g ", m_bodies[i].GetWorldCenter().Y);
			//}

			//for (int i = 0; i < e_count; ++i)
			//{
			//	printf("%g ", m_bodies[i].GetLinearVelocity().Y);
			//}

			//printf("\n");
		}

		public static Test Create()
		{
			return new SphereStack();
		}

		Body[] m_bodies = new Body[e_count];
	};
}
