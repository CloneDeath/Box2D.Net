using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;

namespace Testbed.Tests {
	class SphereStack : Test
	{
		public enum
		{
			e_count = 10
		};

		public SphereStack()
		{
			{
				b2BodyDef bd;
				b2Body* ground = m_world.CreateBody(&bd);

				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(&shape, 0.0f);
			}

			{
				b2CircleShape shape;
				shape.m_radius = 1.0f;

				for (int i = 0; i < e_count; ++i)
				{
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position.Set(0.0, 4.0f + 3.0f * i);

					m_bodies[i] = m_world.CreateBody(&bd);

					m_bodies[i].CreateFixture(&shape, 1.0f);

					m_bodies[i].SetLinearVelocity(b2Vec2(0.0f, -50.0f));
				}
			}
		}

		public void Step(Settings* settings)
		{
			Test::Step(settings);

			//for (int i = 0; i < e_count; ++i)
			//{
			//	printf("%g ", m_bodies[i].GetWorldCenter().y);
			//}

			//for (int i = 0; i < e_count; ++i)
			//{
			//	printf("%g ", m_bodies[i].GetLinearVelocity().y);
			//}

			//printf("\n");
		}

		public static Test Create()
		{
			return new SphereStack();
		}

		b2Body* m_bodies[e_count];
	};
}
