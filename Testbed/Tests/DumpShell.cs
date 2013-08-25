using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	// This test holds worlds dumped using b2World::Dump.
	class DumpShell : Test {
		DumpShell()
		{

		public b2Vec2 g(0.000000000000000e+000f, 0.00000000000000e+00f);
		m_world.SetGravity(g);
		b2Body** bodies = (b2Body**)b2Alloc(2 * sizeof(b2Body*));
		b2Joint** joints = (b2Joint**)b2Alloc(0 * sizeof(b2Joint*));

		{
			b2BodyDef bd;
			bd.type = b2BodyType(2);
			bd.position.Set(3.000000000000000e+001f, 6.909857940673828e+001f);
			bd.angle = 0.000000000000000e+000f;
			bd.linearVelocity.Set(0.000000000000000e+000f, -8.618643951416016e+001f);
			bd.angularVelocity = 0.000000000000000e+000f;
			bd.linearDamping = 0.000000000000000e+000f;
			bd.angularDamping = 0.000000000000000e+000f;
			bd.allowSleep = bool(4);
			bd.awake = bool(2);
			bd.fixedRotation = bool(16);
			bd.bullet = bool(0);
			bd.active = bool(32);
			bd.gravityScale = 1.000000000000000e+000f;
			bodies[0] = m_world.CreateBody(bd);
			{
				b2FixtureDef fd;
				fd.friction = 6.000000238418579e-001f;
				fd.restitution = 0.000000000000000e+000f;
				fd.density = 5.000000000000000e-001f;
				fd.isSensor = bool(0);
				fd.filter.categoryBits = ushort(1);
				fd.filter.maskBits = ushort(2);
				fd.filter.groupIndex = short(0);
				b2PolygonShape shape;
				b2Vec2 vs[8];
				vs[0].Set(-1.950000047683716e+000f, -4.750000000000000e+000f);
				vs[1].Set(1.950000047683716e+000f, -4.750000000000000e+000f);
				vs[2].Set(1.950000047683716e+000f, 4.750000000000000e+000f);
				vs[3].Set(-1.950000047683716e+000f, 4.750000000000000e+000f);
				shape.Set(vs, 4);
		
				fd.shape = &shape;
		
				bodies[0].CreateFixture(&fd);
			}
		}
		
			{
			  b2BodyDef bd;
			  bd.type = b2BodyType(0);
			  bd.position.Set(5.120000457763672e+001f, 7.580000305175781e+001f);
			  bd.angle = 0.000000000000000e+000f;
			  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
			  bd.angularVelocity = 0.000000000000000e+000f;
			  bd.linearDamping = 0.000000000000000e+000f;
			  bd.angularDamping = 0.000000000000000e+000f;
			  bd.allowSleep = bool(4);
			  bd.awake = bool(2);
			  bd.fixedRotation = bool(0);
			  bd.bullet = bool(0);
			  bd.active = bool(32);
			  bd.gravityScale = 1.000000000000000e+000f;
			  bodies[1] = m_world.CreateBody(bd);
			  {
				b2FixtureDef fd;
				fd.friction = 0.000000000000000e+000f;
				fd.restitution = 0.000000000000000e+000f;
				fd.density = 1.000000000000000e+000f;
				fd.isSensor = bool(0);
				fd.filter.categoryBits = ushort(2);
				fd.filter.maskBits = ushort(65535);
				fd.filter.groupIndex = short(0);
				b2PolygonShape shape;
				b2Vec2 vs[8];
				vs[0].Set(-5.120000076293945e+001f, -5.000000000000000e-001f);
				vs[1].Set(5.120000076293945e+001f, -5.000000000000000e-001f);
				vs[2].Set(5.120000076293945e+001f, 5.000000000000000e-001f);
				vs[3].Set(-5.120000076293945e+001f, 5.000000000000000e-001f);
				shape.Set(vs, 4);

				fd.shape = &shape;

				bodies[1].CreateFixture(&fd);
			  }
			}

			b2Free(joints);
			b2Free(bodies);
			joints = null;
			bodies = null;

		}

		public static Test Create()
		{
			return new DumpShell();
		}
	};
}
