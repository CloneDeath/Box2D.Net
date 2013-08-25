using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using Box2D;

namespace Testbed.Tests {
	class CollisionFiltering : Test {
		// This is a test of collision filtering.
		// There is a triangle, a box, and a circle.
		// There are 6 shapes. 3 large and 3 small.
		// The 3 small ones always collide.
		// The 3 large ones never collide.
		// The boxes don't collide with triangles (except if both are small).
		const short k_smallGroup = 1;
		const short k_largeGroup = -1;

		const ushort k_defaultCategory = 0x0001;
		const ushort k_triangleCategory = 0x0002;
		const ushort k_boxCategory = 0x0004;
		const ushort k_circleCategory = 0x0008;

		const ushort k_triangleMask = 0xFFFF;
		const ushort k_boxMask = 0xFFFF ^ k_triangleCategory;
		const ushort k_circleMask = 0xFFFF;

		public CollisionFiltering()
		{
			// Ground body
			{
				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));

				FixtureDef sd = new FixtureDef();
				sd.shape = shape;
				sd.friction = 0.3f;

				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);
				ground.CreateFixture(sd);
			}

			// Small triangle
			Vec2[] vertices = new Vec2[3];
			vertices[0].Set(-1.0f, 0.0f);
			vertices[1].Set(1.0f, 0.0f);
			vertices[2].Set(0.0f, 2.0f);
			PolygonShape polygon = new PolygonShape();
			polygon.Set(vertices, 3);

			FixtureDef triangleShapeDef = new FixtureDef();
			triangleShapeDef.shape = polygon;
			triangleShapeDef.Density = 1.0f;

			triangleShapeDef.Filter.GroupIndex = k_smallGroup;
			triangleShapeDef.Filter.CategoryBits = k_triangleCategory;
			triangleShapeDef.Filter.MaskBits = k_triangleMask;

			BodyDef triangleBodyDef = new BodyDef();
			triangleBodyDef.type = BodyType._dynamicBody;
			triangleBodyDef.Position.Set(-5.0f, 2.0f);

			Body body1 = m_world.CreateBody(triangleBodyDef);
			body1.CreateFixture(triangleShapeDef);

			// Large triangle (recycle definitions)
			vertices[0] *= 2.0f;
			vertices[1] *= 2.0f;
			vertices[2] *= 2.0f;
			polygon.Set(vertices, 3);
			triangleShapeDef.Filter.GroupIndex = k_largeGroup;
			triangleBodyDef.Position.Set(-5.0f, 6.0f);
			triangleBodyDef.fixedRotation = true; // look at me!

			Body body2 = m_world.CreateBody(triangleBodyDef);
			body2.CreateFixture(triangleShapeDef);

			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(-5.0f, 10.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape p = new PolygonShape();
				p.SetAsBox(0.5f, 1.0f);
				body.CreateFixture(p);

				PrismaticJointDef jd = new PrismaticJointDef();
				jd.bodyA = body2;
				jd.bodyB = body;
				jd.enableLimit = true;
				jd.localAnchorA.Set(0.0f, 4.0f);
				jd.localAnchorB.SetZero();
				jd.localAxisA.Set(0.0f, 1.0f);
				jd.lowerTranslation = -1.0f;
				jd.upperTranslation = 1.0f;

				m_world.CreateJoint(jd);
			}

			// Small box
			polygon.SetAsBox(1.0f, 0.5f);
			FixtureDef boxShapeDef = new FixtureDef();
			boxShapeDef.shape = polygon;
			boxShapeDef.Density = 1.0f;
			boxShapeDef.restitution = 0.1f;

			boxShapeDef.Filter.GroupIndex = k_smallGroup;
			boxShapeDef.Filter.CategoryBits = k_boxCategory;
			boxShapeDef.Filter.MaskBits = k_boxMask;

			BodyDef boxBodyDef = new BodyDef();
			boxBodyDef.type = BodyType._dynamicBody;
			boxBodyDef.Position.Set(0.0f, 2.0f);

			Body body3 = m_world.CreateBody(boxBodyDef);
			body3.CreateFixture(boxShapeDef);

			// Large box (recycle definitions)
			polygon.SetAsBox(2.0f, 1.0f);
			boxShapeDef.Filter.GroupIndex = k_largeGroup;
			boxBodyDef.Position.Set(0.0f, 6.0f);

			Body body4 = m_world.CreateBody(boxBodyDef);
			body4.CreateFixture(boxShapeDef);

			// Small circle
			CircleShape circle = new CircleShape();
			circle.m_radius = 1.0f;

			FixtureDef circleShapeDef = new FixtureDef();
			circleShapeDef.shape = circle;
			circleShapeDef.Density = 1.0f;

			circleShapeDef.Filter.GroupIndex = k_smallGroup;
			circleShapeDef.Filter.CategoryBits = k_circleCategory;
			circleShapeDef.Filter.MaskBits = k_circleMask;

			BodyDef circleBodyDef = new BodyDef();
			circleBodyDef.type = BodyType._dynamicBody;
			circleBodyDef.Position.Set(5.0f, 2.0f);
		
			Body body5 = m_world.CreateBody(circleBodyDef);
			body5.CreateFixture(circleShapeDef);

			// Large circle
			circle.m_radius *= 2.0f;
			circleShapeDef.Filter.GroupIndex = k_largeGroup;
			circleBodyDef.Position.Set(5.0f, 6.0f);

			Body body6 = m_world.CreateBody(circleBodyDef);
			body6.CreateFixture(circleShapeDef);
		}

		public static Test Create()
		{
			return new CollisionFiltering();
		}
	}
}
