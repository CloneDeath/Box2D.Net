using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class AddPair : Test
	{
		public AddPair()
		{
			m_world.SetGravity(new b2Vec2(0.0f,0.0f));
			{
				b2CircleShape shape;
				shape.m_p.SetZero();
				shape.m_radius = 0.1f;

				float minX = -6.0f;
				float maxX = 0.0f;
				float minY = 4.0f;
				float maxY = 6.0f;
			
				for (int i = 0; i < 400; ++i)
				{
					b2BodyDef bd;
					bd.type = b2BodyType.b2_dynamicBody;
					bd.position = new b2Vec2(RandomFloat(minX,maxX),RandomFloat(minY,maxY));
					b2Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 0.01f);
				}
			}
		
			{
				b2PolygonShape shape;
				shape.SetAsBox(1.5f, 1.5f);
				b2BodyDef bd;
				bd.type = b2BodyType.b2_dynamicBody;
				bd.position.Set(-40.0f,5.0f);
				bd.bullet = true;
				b2Body body = m_world.CreateBody(bd);
				body.CreateFixture(shape, 1.0f);
				body.SetLinearVelocity(b2Vec2(150.0f, 0.0f));
			}
		}

		public static Test Create()
		{
			return new AddPair();
		}
	};
}
