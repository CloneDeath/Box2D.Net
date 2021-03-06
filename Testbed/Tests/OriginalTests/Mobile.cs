﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class Mobile : Test
	{
		const int e_depth = 4;

		public Mobile()
		{
			Body ground;

			// Create ground body.
			{
				BodyDef bodyDef = new BodyDef();
				bodyDef.Position.Set(0.0f, 20.0f);
				ground = m_world.CreateBody(bodyDef);
			}

			float a = 0.5f;
			Vec2 h = new Vec2(0.0f, a);

			Body root = AddNode(ground, new Vec2(0, 0), 0, 3.0f, a);

			RevoluteJointDef jointDef = new RevoluteJointDef();
			jointDef.bodyA = ground;
			jointDef.bodyB = root;
			jointDef.localAnchorA.SetZero();
			jointDef.localAnchorB = h;
			m_world.CreateJoint(jointDef);
		}

		public Body AddNode(Body parent, Vec2 localAnchor, int depth, float offset, float a)
		{
			Vec2 h = new Vec2(0.0f, a);

			Vec2 p = parent.GetPosition() + localAnchor - h;

			BodyDef bodyDef = new BodyDef();
			bodyDef.type = BodyType._dynamicBody;
			bodyDef.Position = p;
			Body body = m_world.CreateBody(bodyDef);

			PolygonShape shape = new PolygonShape();
			shape.SetAsBox(0.25f * a, a);
			shape.Density = 20;
			body.CreateFixture(shape);

			if (depth == e_depth)
			{
				return body;
			}

			Vec2 a1 = new Vec2(offset, -a);
			Vec2 a2 = new Vec2(-offset, -a);
			Body body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a);
			Body body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a);

			RevoluteJointDef jointDef = new RevoluteJointDef();
			jointDef.bodyA = body;
			jointDef.localAnchorB = h;

			jointDef.localAnchorA = a1;
			jointDef.bodyB = body1;
			m_world.CreateJoint(jointDef);

			jointDef.localAnchorA = a2;
			jointDef.bodyB = body2;
			m_world.CreateJoint(jointDef);

			return body;
		}

		public static Test Create()
		{
			return new Mobile();
		}
	};
}
