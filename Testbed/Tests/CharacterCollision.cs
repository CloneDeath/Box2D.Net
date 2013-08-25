using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	/// This is a test of typical character collision scenarios. This does not
	/// show how you should implement a character in your application.
	/// Instead this is used to test smooth collision on edge chains.
	class CharacterCollision : Test
	{
		public CharacterCollision()
		{
			// Ground body
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Collinear edges with no adjacency information.
			// This shows the problematic case where a box shape can hit
			// an internal vertex.
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Density = 0;
				shape.Set(new Vec2(-8.0f, 1.0f), new Vec2(-6.0f, 1.0f));
				ground.CreateFixture(shape);
				shape.Set(new Vec2(-6.0f, 1.0f), new Vec2(-4.0f, 1.0f));
				ground.CreateFixture(shape);
				shape.Set(new Vec2(-4.0f, 1.0f), new Vec2(-2.0f, 1.0f));
				ground.CreateFixture(shape);
			}

			// Chain shape
			{
				BodyDef bd = new BodyDef();
				bd.angle = 0.25f * (float)Math.PI;
				Body ground = m_world.CreateBody(bd);

				Vec2[] vs = new Vec2[4];
				vs[0].Set(5.0f, 7.0f);
				vs[1].Set(6.0f, 8.0f);
				vs[2].Set(7.0f, 8.0f);
				vs[3].Set(8.0f, 7.0f);
				ChainShape shape = new ChainShape();
				shape.CreateChain(vs, 4);
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Square tiles. This shows that adjacency shapes may
			// have non-smooth collision. There is no solution
			// to this problem.
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.Density = 0;

				shape.SetAsBox(1.0f, 1.0f, new Vec2(4.0f, 3.0f), 0.0f);
				ground.CreateFixture(shape);
				shape.SetAsBox(1.0f, 1.0f, new Vec2(6.0f, 3.0f), 0.0f);
				ground.CreateFixture(shape);
				shape.SetAsBox(1.0f, 1.0f, new Vec2(8.0f, 3.0f), 0.0f);
				ground.CreateFixture(shape);
			}

			// Square made from an edge loop. Collision should be smooth.
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				Vec2[] vs = new Vec2[4];
				vs[0].Set(-1.0f, 3.0f);
				vs[1].Set(1.0f, 3.0f);
				vs[2].Set(1.0f, 5.0f);
				vs[3].Set(-1.0f, 5.0f);
				ChainShape shape = new ChainShape();
				shape.CreateLoop(vs, 4);
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Edge loop. Collision should be smooth.
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(-10.0f, 4.0f);
				Body ground = m_world.CreateBody(bd);

				Vec2[] vs = new Vec2[10];
				vs[0].Set(0.0f, 0.0f);
				vs[1].Set(6.0f, 0.0f);
				vs[2].Set(6.0f, 2.0f);
				vs[3].Set(4.0f, 1.0f);
				vs[4].Set(2.0f, 2.0f);
				vs[5].Set(0.0f, 2.0f);
				vs[6].Set(-2.0f, 2.0f);
				vs[7].Set(-4.0f, 3.0f);
				vs[8].Set(-6.0f, 2.0f);
				vs[9].Set(-6.0f, 0.0f);
				ChainShape shape = new ChainShape();
				shape.CreateLoop(vs, 10);
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			// Square character 1
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(-3.0f, 8.0f);
				bd.type = BodyType._dynamicBody;
				bd.fixedRotation = true;
				bd.allowSleep = false;

				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				body.CreateFixture(fd);
			}

			// Square character 2
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(-5.0f, 5.0f);
				bd.type = BodyType._dynamicBody;
				bd.fixedRotation = true;
				bd.allowSleep = false;

				Body body = m_world.CreateBody(bd);

				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.25f, 0.25f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				body.CreateFixture(fd);
			}

			// Hexagon character
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(-5.0f, 8.0f);
				bd.type = BodyType._dynamicBody;
				bd.fixedRotation = true;
				bd.allowSleep = false;

				Body body = m_world.CreateBody(bd);

				float angle = 0.0f;
				float delta = (float)Math.PI / 3.0f;
				Vec2[] vertices = new Vec2[6];
				for (int i = 0; i < 6; ++i)
				{
					vertices[i].Set(0.5f * (float)Math.Cos(angle), 0.5f * (float)Math.Sin(angle));
					angle += delta;
				}

				PolygonShape shape = new PolygonShape();
				shape.Set(vertices, 6);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				body.CreateFixture(fd);
			}

			// Circle character
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(3.0f, 5.0f);
				bd.type = BodyType._dynamicBody;
				bd.fixedRotation = true;
				bd.allowSleep = false;

				Body body = m_world.CreateBody(bd);

				CircleShape shape = new CircleShape();
				shape.m_radius = 0.5f;

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				body.CreateFixture(fd);
			}

			// Circle character
			{
				BodyDef bd = new BodyDef();
				bd.Position.Set(-7.0f, 6.0f);
				bd.type = BodyType._dynamicBody;
				bd.allowSleep = false;

				m_character = m_world.CreateBody(bd);

				CircleShape shape = new CircleShape();
				shape.m_radius = 0.25f;

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 20.0f;
				fd.friction = 1.0f;
				m_character.CreateFixture(fd);
			}
		}

		public override void Step(TestSettings settings)
		{
			Vec2 v = m_character.GetLinearVelocity();
			v.X = -5.0f;
			m_character.SetLinearVelocity(v);

			base.Step(settings);
			m_debugDraw.DrawString("This tests various character collision shapes.");
			
			m_debugDraw.DrawString("Limitation: square and hexagon can snag on aligned boxes.");
			
			m_debugDraw.DrawString("Feature: edge chains have smooth collision inside and out.");
			
		}

		public static Test Create()
		{
			return new CharacterCollision();
		}

		Body m_character;
	};
}
