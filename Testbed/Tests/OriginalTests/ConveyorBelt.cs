﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	class ConveyorBelt : Test
	{
		public ConveyorBelt()
		{
			// Ground
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Platform
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(-5.0f, 5.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(10.0f, 0.5f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.friction = 0.8f;
				m_platform = body.CreateFixture(fd);
			}

			// Boxes
			for (int i = 0; i < 5; ++i)
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;
				bd.Position.Set(-10.0f + 2.0f * i, 7.0f);
				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);
				shape.Density = 20;
				body.CreateFixture(shape);
			}
		}

		public override void PreSolve(Contact contact, Manifold oldManifold)
		{
			base.PreSolve(contact, oldManifold);

			Fixture fixtureA = contact.FixtureA;
			Fixture fixtureB = contact.FixtureB;

			if (fixtureA == m_platform)
			{
				contact.SetTangentSpeed(5.0f);
			}

			if (fixtureB == m_platform)
			{
				contact.SetTangentSpeed(-5.0f);
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);
		}

		public static Test Create()
		{
			return new ConveyorBelt();
		}

		Fixture m_platform;
	};
}
