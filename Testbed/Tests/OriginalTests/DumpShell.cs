﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// This test holds worlds dumped using World::Dump.
	class DumpShell : Test {
		DumpShell() {
			Vec2 g = new Vec2(0, 0);
			m_world.SetGravity(g);
			List<Body> bodies = new List<Body>();
			List<Joint> joints = new List<Joint>();

			{
				BodyDef bd = new BodyDef();
				bd.type = (BodyType)2;
				bd.Position.Set(3.000000000000000e+001f, 6.909857940673828e+001f);
				bd.angle = 0.000000000000000e+000f;
				bd.linearVelocity.Set(0.000000000000000e+000f, -8.618643951416016e+001f);
				bd.angularVelocity = 0.000000000000000e+000f;
				bd.linearDamping = 0.000000000000000e+000f;
				bd.angularDamping = 0.000000000000000e+000f;
				bd.allowSleep = true;
				bd.awake = true;
				bd.fixedRotation = true;
				bd.bullet = false;
				bd.active = true;
				bd.gravityScale = 1.000000000000000e+000f;
				bodies.Add(m_world.CreateBody(bd));
				{
					FixtureDef fd = new FixtureDef();
					fd.friction = 6.000000238418579e-001f;
					fd.restitution = 0.000000000000000e+000f;
					fd.Density = 5.000000000000000e-001f;
					fd.IsSensor = false;
					fd.Filter.CategoryBits = (ushort)(1);
					fd.Filter.MaskBits = (ushort)(2);
					fd.Filter.GroupIndex = (short)(0);
					PolygonShape shape = new PolygonShape();
					Vec2[] vs = new Vec2[8];
					vs[0].Set(-1.950000047683716e+000f, -4.750000000000000e+000f);
					vs[1].Set(1.950000047683716e+000f, -4.750000000000000e+000f);
					vs[2].Set(1.950000047683716e+000f, 4.750000000000000e+000f);
					vs[3].Set(-1.950000047683716e+000f, 4.750000000000000e+000f);
					shape.Set(vs, 4);
		
					fd.shape = shape;
		
					bodies[0].CreateFixture(fd);
				}
			}
		
			{
				BodyDef bd = new BodyDef();
				bd.type = (BodyType)(0);
				bd.Position.Set(5.120000457763672e+001f, 7.580000305175781e+001f);
				bd.angle = 0.000000000000000e+000f;
				bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
				bd.angularVelocity = 0.000000000000000e+000f;
				bd.linearDamping = 0.000000000000000e+000f;
				bd.angularDamping = 0.000000000000000e+000f;
				bd.allowSleep = true;
				bd.awake = true;
				bd.fixedRotation = false;
				bd.bullet = false;
				bd.active = true;
				bd.gravityScale = 1.000000000000000e+000f;
				bodies.Add(m_world.CreateBody(bd));
				{
					FixtureDef fd = new FixtureDef();
					fd.friction = 0.000000000000000e+000f;
					fd.restitution = 0.000000000000000e+000f;
					fd.Density = 1.000000000000000e+000f;
					fd.IsSensor = false;
					fd.Filter.CategoryBits = (ushort)(2);
					fd.Filter.MaskBits = (ushort)(65535);
					fd.Filter.GroupIndex = (short)(0);
					PolygonShape shape = new PolygonShape();
					Vec2[] vs = new Vec2[8];
					vs[0].Set(-5.120000076293945e+001f, -5.000000000000000e-001f);
					vs[1].Set(5.120000076293945e+001f, -5.000000000000000e-001f);
					vs[2].Set(5.120000076293945e+001f, 5.000000000000000e-001f);
					vs[3].Set(-5.120000076293945e+001f, 5.000000000000000e-001f);
					shape.Set(vs, 4);

					fd.shape = shape;

					bodies[1].CreateFixture(fd);
				}
			}

			joints = null;
			bodies = null;
		}

		public static Test Create() {
			return new DumpShell();
		}
	};
}
