using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using System.Drawing;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class DynamicTreeTest : Test
	{
		const int e_actorCount = 128;
		Random rand;

		public DynamicTreeTest()
		{
			m_worldExtent = 15.0f;
			m_proxyExtent = 0.5f;

			rand = new Random(888);

			for (int i = 0; i < e_actorCount; ++i)
			{
				Actor actor = m_actors[i];
				GetRandomAABB(actor.aabb);
				actor.proxyId = m_tree.CreateProxy(actor.aabb, actor);
			}

			m_stepCount = 0;

			float h = m_worldExtent;
			m_queryAABB.lowerBound.Set(-3.0f, -4.0f + h);
			m_queryAABB.upperBound.Set(5.0f, 6.0f + h);

			m_rayCastInput.p1.Set(-5.0f, 5.0f + h);
			m_rayCastInput.p2.Set(7.0f, -4.0f + h);
			//m_rayCastInput.p1.Set(0.0f, 2.0f + h);
			//m_rayCastInput.p2.Set(0.0f, -2.0f + h);
			m_rayCastInput.maxFraction = 1.0f;

			m_automated = false;
		}

		public static Test Create()
		{
			return new DynamicTreeTest();
		}

		public void Step(TestSettings settings)
		{
			m_rayActor = null;
			for (int i = 0; i < e_actorCount; ++i)
			{
				m_actors[i].fraction = 1.0f;
				m_actors[i].overlap = false;
			}

			if (m_automated == true)
			{
				int actionCount = Math.Max(1, e_actorCount >> 2);

				for (int i = 0; i < actionCount; ++i)
				{
					Action();
				}
			}

			Query();
			RayCast();

			for (int i = 0; i < e_actorCount; ++i)
			{
				Actor actor = m_actors[i];
				if (actor.proxyId == TreeNode._nullNode)
					continue;

				Color cv = Color.FromArgb(225, 225, 225);
				if (actor == m_rayActor && actor.overlap)
				{
					cv = Color.FromArgb(225, 150, 150);
				}
				else if (actor == m_rayActor)
				{
					cv = Color.FromArgb(150, 225, 150);
				}
				else if (actor.overlap)
				{
					cv = Color.FromArgb(150, 150, 225);
				}

				m_debugDraw.DrawAABB(actor.aabb, cv);
			}

			Color c = Color.FromArgb(175, 175, 175);
			m_debugDraw.DrawAABB(m_queryAABB, c);

			m_debugDraw.DrawSegment(m_rayCastInput.p1, m_rayCastInput.p2, c);

			Color c1 = Color.FromArgb(50, 225, 50);
			Color c2 = Color.FromArgb(225, 50, 50);
			m_debugDraw.DrawPoint(m_rayCastInput.p1, 6.0f, c1);
			m_debugDraw.DrawPoint(m_rayCastInput.p2, 6.0f, c2);

			if (m_rayActor != null)
			{
				Color cr = Color.FromArgb(50, 50, 225);
				Vec2 p = m_rayCastInput.p1 + m_rayActor.fraction * (m_rayCastInput.p2 - m_rayCastInput.p1);
				m_debugDraw.DrawPoint(p, 6.0f, cr);
			}

			{
				int height = m_tree.GetHeight();
				m_debugDraw.DrawString("dynamic tree height = %d", height);
				
			}

			++m_stepCount;
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.A)){
				m_automated = !m_automated;
			}

			if (KeyboardManager.IsPressed(Key.C)){
				CreateProxy();
			}

			if (KeyboardManager.IsPressed(Key.D)){
				DestroyProxy();
			}

			if (KeyboardManager.IsPressed(Key.M)) {
				MoveProxy();
			}
		}

		public bool QueryCallback(int proxyId)
		{
			Actor actor = (Actor)m_tree.GetUserData(proxyId);
			actor.overlap = Collision.TestOverlap(m_queryAABB, actor.aabb);
			return true;
		}

		public float RayCastCallback(RayCastInput input, int proxyId)
		{
			Actor actor = (Actor)m_tree.GetUserData(proxyId);

			RayCastOutput output;
			bool hit = actor.aabb.RayCast(out output, input);

			if (hit)
			{
				m_rayCastOutput = output;
				m_rayActor = actor;
				m_rayActor.fraction = output.fraction;
				return output.fraction;
			}

			return input.maxFraction;
		}

		private class Actor
		{
			public AABB aabb;
			public float fraction;
			public bool overlap;
			public int proxyId;
		};

		private void GetRandomAABB(AABB aabb)
		{
			Vec2 w = new Vec2(); 
			w.Set(2.0f * m_proxyExtent, 2.0f * m_proxyExtent);
			//aabb.lowerBound.X = -m_proxyExtent;
			//aabb.lowerBound.Y = -m_proxyExtent + m_worldExtent;
			aabb.lowerBound.X = RandomFloat(-m_worldExtent, m_worldExtent);
			aabb.lowerBound.Y = RandomFloat(0.0f, 2.0f * m_worldExtent);
			aabb.upperBound = aabb.lowerBound + w;
		}

		private void MoveAABB(AABB aabb)
		{
			Vec2 d;
			d.X = RandomFloat(-0.5f, 0.5f);
			d.Y = RandomFloat(-0.5f, 0.5f);
			//d.X = 2.0f;
			//d.Y = 0.0f;
			aabb.lowerBound += d;
			aabb.upperBound += d;

			Vec2 c0 = 0.5f * (aabb.lowerBound + aabb.upperBound);
			Vec2 min = new Vec2(); min.Set(-m_worldExtent, 0.0f);
			Vec2 max = new Vec2(); max.Set(m_worldExtent, 2.0f * m_worldExtent);
			Vec2 c = Utilities.Clamp(c0, min, max);

			aabb.lowerBound += c - c0;
			aabb.upperBound += c - c0;
		}

		private void CreateProxy()
		{
			for (int i = 0; i < e_actorCount; ++i)
			{
				int j = rand.Next(e_actorCount);
				Actor actor = m_actors[j];
				if (actor.proxyId == TreeNode._nullNode)
				{
					GetRandomAABB(actor.aabb);
					actor.proxyId = m_tree.CreateProxy(actor.aabb, actor);
					return;
				}
			}
		}

		private void DestroyProxy()
		{
			for (int i = 0; i < e_actorCount; ++i)
			{
				int j = rand.Next(e_actorCount);
				Actor actor = m_actors[j];
				if (actor.proxyId != TreeNode._nullNode)
				{
					m_tree.DestroyProxy(actor.proxyId);
					actor.proxyId = TreeNode._nullNode;
					return;
				}
			}
		}

		private void MoveProxy()
		{
			for (int i = 0; i < e_actorCount; ++i)
			{
				int j = rand.Next(e_actorCount);
				Actor actor = m_actors[j];
				if (actor.proxyId == TreeNode._nullNode)
				{
					continue;
				}

				AABB aabb0 = actor.aabb;
				MoveAABB(actor.aabb);
				Vec2 displacement = actor.aabb.GetCenter() - aabb0.GetCenter();
				m_tree.MoveProxy(actor.proxyId, actor.aabb, displacement);
				return;
			}
		}

		private void Action()
		{
			int choice = rand.Next(20);

			switch (choice)
			{
			case 0:
				CreateProxy();
				break;

			case 1:
				DestroyProxy();
				break;

			default:
				MoveProxy();
				break;
			}
		}

		private void Query()
		{
			m_tree.Query(this.QueryCallback, m_queryAABB);

			for (int i = 0; i < e_actorCount; ++i)
			{
				if (m_actors[i].proxyId == TreeNode._nullNode)
				{
					continue;
				}

				bool overlap = Collision.TestOverlap(m_queryAABB, m_actors[i].aabb);
				Utilities.Assert(overlap == m_actors[i].overlap);
			}
		}

		private void RayCast()
		{
			m_rayActor = null;

			RayCastInput input = m_rayCastInput;

			// Ray cast against the dynamic tree.
			m_tree.RayCast(this, input);

			// Brute force ray cast.
			Actor bruteActor = null;
			RayCastOutput bruteOutput = new RayCastOutput();
			for (int i = 0; i < e_actorCount; ++i)
			{
				if (m_actors[i].proxyId == TreeNode._nullNode)
				{
					continue;
				}

				RayCastOutput output;
				bool hit = m_actors[i].aabb.RayCast(out output, input);
				if (hit)
				{
					bruteActor = m_actors[i];
					bruteOutput = output;
					input.maxFraction = output.fraction;
				}
			}

			if (bruteActor != null)
			{
				Utilities.Assert(bruteOutput.fraction == m_rayCastOutput.fraction);
			}
		}

		private float m_worldExtent;
		private float m_proxyExtent;

		private DynamicTree m_tree;
		private AABB m_queryAABB;
		private RayCastInput m_rayCastInput;
		private RayCastOutput m_rayCastOutput;
		private Actor m_rayActor;
		private Actor[] m_actors = new Actor[e_actorCount];
		private int m_stepCount;
		private bool m_automated;
	};
}
