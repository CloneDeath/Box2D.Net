﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class VerticalStack : Test
	{
		const int e_columnCount = 5;
		const int e_rowCount = 16;
			//e_columnCount = 1,
			//e_rowCount = 1

		public VerticalStack()
		{
			{
				BodyDef bd = new BodyDef();
				Body ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);

				shape.Set(new Vec2(20.0f, 0.0f), new Vec2(20.0f, 20.0f));
				shape.Density = 0;
				ground.CreateFixture(shape);
			}

			float[] xs = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};

			for (int j = 0; j < e_columnCount; ++j)
			{
				PolygonShape shape = new PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				fd.Density = 1.0f;
				fd.friction = 0.3f;

				for (int i = 0; i < e_rowCount; ++i)
				{
					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;

					int n = j * e_rowCount + i;
					Utilities.Assert(n < e_rowCount * e_columnCount);
					m_indices[n] = n;
					bd.UserData = m_indices[n];

					float x = 0.0f;
					//float x = RandomFloat(-0.02f, 0.02f);
					//float x = i % 2 == 0 ? -0.025f : 0.025f;
					bd.Position.Set(xs[j] + x, 0.752f + 1.54f * i);
					Body body = m_world.CreateBody(bd);

					m_bodies[n] = body;

					body.CreateFixture(fd);
				}
			}

			m_bullet = null;
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.Comma)) {
				if (m_bullet != null)
				{
					m_world.DestroyBody(m_bullet);
					m_bullet = null;
				}

				{
					CircleShape shape = new CircleShape();
					shape.m_radius = 0.25f;

					FixtureDef fd = new FixtureDef();
					fd.shape = shape;
					fd.Density = 20.0f;
					fd.restitution = 0.05f;

					BodyDef bd = new BodyDef();
					bd.type = BodyType._dynamicBody;
					bd.bullet = true;
					bd.Position.Set(-31.0f, 5.0f);

					m_bullet = m_world.CreateBody(bd);
					m_bullet.CreateFixture(fd);

					m_bullet.SetLinearVelocity(new Vec2(400.0f, 0.0f));
				}
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Press: (,) to launch a bullet.");
			

			//if (m_stepCount == 300)
			//{
			//	if (m_bullet != null)
			//	{
			//		m_world.DestroyBody(m_bullet);
			//		m_bullet = null;
			//	}

			//	{
			//		CircleShape shape = new CircleShape();
			//		shape.m_radius = 0.25f;

			//		FixtureDef fd = new FixtureDef();
			//		fd.shape = shape;
			//		fd.Density = 20.0f;
			//		fd.restitution = 0.05f;

			//		BodyDef bd = new BodyDef();
			//		bd.type = BodyType._dynamicBody;
			//		bd.bullet = true;
			//		bd.position.Set(-31.0f, 5.0f);

			//		m_bullet = m_world.CreateBody(bd);
			//		m_bullet.CreateFixture(fd);

			//		m_bullet.SetLinearVelocity(new Vec2(400.0f, 0.0f));
			//	}
			//}
		}

		public static Test Create()
		{
			return new VerticalStack();
		}

		Body m_bullet;
		Body[] m_bodies = new Body[e_rowCount * e_columnCount];
		int[] m_indices = new int[e_rowCount * e_columnCount];
	};
}
