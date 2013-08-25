using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	// This tests distance joints, body destruction, and joint destruction.
	class Web : Test
	{
		public Web()
		{
			b2Body ground = null;
			{
				b2BodyDef bd = new b2BodyDef();
				ground = m_world.CreateBody(bd);

				b2EdgeShape shape = new b2EdgeShape();
				shape.Set(new b2Vec2(-40.0f, 0.0f), new b2Vec2(40.0f, 0.0f));
				ground.CreateFixture(shape, 0.0f);
			}

			{
				b2PolygonShape shape = new b2PolygonShape();
				shape.SetAsBox(0.5f, 0.5f);

				b2BodyDef bd = new b2BodyDef();
				bd.type = b2BodyType.b2_dynamicBody;

				bd.position.Set(-5.0f, 5.0f);
				m_bodies[0] = m_world.CreateBody(bd);
				m_bodies[0].CreateFixture(shape, 5.0f);

				bd.position.Set(5.0f, 5.0f);
				m_bodies[1] = m_world.CreateBody(bd);
				m_bodies[1].CreateFixture(shape, 5.0f);

				bd.position.Set(5.0f, 15.0f);
				m_bodies[2] = m_world.CreateBody(bd);
				m_bodies[2].CreateFixture(shape, 5.0f);

				bd.position.Set(-5.0f, 15.0f);
				m_bodies[3] = m_world.CreateBody(bd);
				m_bodies[3].CreateFixture(shape, 5.0f);

				b2DistanceJointDef jd = new b2DistanceJointDef();
				b2Vec2 p1, p2, d;

				jd.frequencyHz = 2.0f;
				jd.dampingRatio = 0.0f;

				jd.bodyA = ground;
				jd.bodyB = m_bodies[0];
				jd.localAnchorA.Set(-10.0f, 0.0f);
				jd.localAnchorB.Set(-0.5f, -0.5f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[0] = m_world.CreateJoint(jd);

				jd.bodyA = ground;
				jd.bodyB = m_bodies[1];
				jd.localAnchorA.Set(10.0f, 0.0f);
				jd.localAnchorB.Set(0.5f, -0.5f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[1] = m_world.CreateJoint(jd);

				jd.bodyA = ground;
				jd.bodyB = m_bodies[2];
				jd.localAnchorA.Set(10.0f, 20.0f);
				jd.localAnchorB.Set(0.5f, 0.5f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[2] = m_world.CreateJoint(jd);

				jd.bodyA = ground;
				jd.bodyB = m_bodies[3];
				jd.localAnchorA.Set(-10.0f, 20.0f);
				jd.localAnchorB.Set(-0.5f, 0.5f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[3] = m_world.CreateJoint(jd);

				jd.bodyA = m_bodies[0];
				jd.bodyB = m_bodies[1];
				jd.localAnchorA.Set(0.5f, 0.0f);
				jd.localAnchorB.Set(-0.5f, 0.0f);;
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[4] = m_world.CreateJoint(jd);

				jd.bodyA = m_bodies[1];
				jd.bodyB = m_bodies[2];
				jd.localAnchorA.Set(0.0f, 0.5f);
				jd.localAnchorB.Set(0.0f, -0.5f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[5] = m_world.CreateJoint(jd);

				jd.bodyA = m_bodies[2];
				jd.bodyB = m_bodies[3];
				jd.localAnchorA.Set(-0.5f, 0.0f);
				jd.localAnchorB.Set(0.5f, 0.0f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[6] = m_world.CreateJoint(jd);

				jd.bodyA = m_bodies[3];
				jd.bodyB = m_bodies[0];
				jd.localAnchorA.Set(0.0f, -0.5f);
				jd.localAnchorB.Set(0.0f, 0.5f);
				p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
				p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
				d = p2 - p1;
				jd.length = d.Length();
				m_joints[7] = m_world.CreateJoint(jd);
			}
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.B)){
				for (int i = 0; i < 4; ++i)
				{
					if (m_bodies[i] != null)
					{
						m_world.DestroyBody(m_bodies[i]);
						m_bodies[i] = null;
						break;
					}
				}
			}

			if (KeyboardManager.IsPressed(Key.J)){
				for (int i = 0; i < 8; ++i)
				{
					if (m_joints[i] != null)
					{
						m_world.DestroyJoint(m_joints[i]);
						m_joints[i] = null;
						break;
					}
				}
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("This demonstrates a soft distance joint.");
			
			m_debugDraw.DrawString("Press: (b) to delete a body, (j) to delete a joint");
			
		}

		public override void JointDestroyed(b2Joint joint)
		{
			for (int i = 0; i < 8; ++i)
			{
				if (m_joints[i] == joint)
				{
					m_joints[i] = null;
					break;
				}
			}
		}

		public static Test Create()
		{
			return new Web();
		}

		b2Body[] m_bodies = new b2Body[4];
		b2Joint[] m_joints = new b2Joint[8];
	};
}
