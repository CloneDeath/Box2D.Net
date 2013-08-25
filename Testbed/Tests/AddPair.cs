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
			m_world.SetGravity(new Vec2(0.0f,0.0f));
			{
				CircleShape shape = new CircleShape();
				shape.m_p.SetZero();
				shape.m_radius = 0.1f;

				float minX = -6.0f;
				float maxX = 0.0f;
				float minY = 4.0f;
				float maxY = 6.0f;
			
				for (int i = 0; i < 400; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.position = new Vec2(RandomFloat(minX,maxX),RandomFloat(minY,maxY));
					Body body = m_world.CreateBody(bd);
					body.CreateFixture(shape, 0.01f);
				}
			}
		
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(1.5f, 1.5f);
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.position.Set(-40.0f,5.0f);
				bd.bullet = true;
				Body body = m_world.CreateBody(bd);
				body.CreateFixture(shape, 1.0f);
				body.SetLinearVelocity(new Vec2(150.0f, 0.0f));
			}
		}

		public static Test Create()
		{
			return new AddPair();
		}
	};
}
