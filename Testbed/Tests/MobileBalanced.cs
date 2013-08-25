using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class MobileBalanced : Test
	{
		public enum
		{
			e_depth = 4
		};

		public MobileBalanced()
		{
			b2Body* ground;

			// Create ground body.
			{
				b2BodyDef bodyDef;
				bodyDef.position.Set(0.0f, 20.0f);
				ground = m_world.CreateBody(&bodyDef);
			}

			float a = 0.5f;
			b2Vec2 h(0.0f, a);

			b2Body* root = AddNode(ground, new b2Vec2(0, 0), 0, 3.0f, a);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = ground;
			jointDef.bodyB = root;
			jointDef.localAnchorA.SetZero();
			jointDef.localAnchorB = h;
			m_world.CreateJoint(&jointDef);
		}

		public b2Body* AddNode(b2Body* parent, b2Vec2 localAnchor, int depth, float offset, float a)
		{
			float density = 20.0f;
			b2Vec2 h(0.0f, a);

			b2Vec2 p = parent.GetPosition() + localAnchor - h;

			b2BodyDef bodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = p;
			b2Body* body = m_world.CreateBody(&bodyDef);

			b2PolygonShape shape;
			shape.SetAsBox(0.25f * a, a);
			body.CreateFixture(&shape, density);

			if (depth == e_depth)
			{
				return body;
			}

			shape.SetAsBox(offset, 0.25f * a, b2Vec2(0, -a), 0.0f);
			body.CreateFixture(&shape, density);

			b2Vec2 a1 = b2Vec2(offset, -a);
			b2Vec2 a2 = b2Vec2(-offset, -a);
			b2Body* body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a);
			b2Body* body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a);

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = body;
			jointDef.localAnchorB = h;

			jointDef.localAnchorA = a1;
			jointDef.bodyB = body1;
			m_world.CreateJoint(&jointDef);

			jointDef.localAnchorA = a2;
			jointDef.bodyB = body2;
			m_world.CreateJoint(&jointDef);

			return body;
		}

		public static Test Create()
		{
			return new MobileBalanced();
		}
	};
}
