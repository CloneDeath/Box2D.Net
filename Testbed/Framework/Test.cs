﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	abstract class Test : b2ContactListener
	{
		static Random rand = new Random();
		/// Random number in range [-1,1]
		public static float RandomFloat()
		{
			return (float)(rand.NextDouble() - 0.5) * 2;
		}
		/// Random floating point number in range [lo, hi]
		public static float RandomFloat(float lo, float hi)
		{
			return (float)((rand.NextDouble() * (hi - lo)) + lo);
		}


		public Test(){
			b2Vec2 gravity;
			gravity.Set(0.0f, -10.0f);
			m_world = new b2World(gravity);
			m_bomb = null;
			m_textLine = 30;
			m_mouseJoint = null;
			m_pointCount = 0;

			m_destructionListener.test = this;
			m_world.SetDestructionListener(&m_destructionListener);
			m_world.SetContactListener(this);
			m_world.SetDebugDraw(&m_debugDraw);
	
			m_bombSpawning = false;

			m_stepCount = 0;

			b2BodyDef bodyDef;
			m_groundBody = m_world.CreateBody(&bodyDef);

			memset(&m_maxProfile, 0, sizeof(b2Profile));
			memset(&m_totalProfile, 0, sizeof(b2Profile));
		}

		public void DrawTitle(string title){
			m_debugDraw.DrawString(5, DRAW_STRING_NEW_LINE, string);
			m_textLine = 2 * DRAW_STRING_NEW_LINE;
		}

		public abstract void Step(Settings settings){
			float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : float(0.0f);

			if (settings.pause)
			{
				if (settings.singleStep)
				{
					settings.singleStep = 0;
				}
				else
				{
					timeStep = 0.0f;
				}

				m_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
				m_textLine += DRAW_STRING_NEW_LINE;
			}

			uint flags = 0;
			flags += settings.drawShapes			* b2Draw::e_shapeBit;
			flags += settings.drawJoints			* b2Draw::e_jointBit;
			flags += settings.drawAABBs			* b2Draw::e_aabbBit;
			flags += settings.drawCOMs				* b2Draw::e_centerOfMassBit;
			m_debugDraw.SetFlags(flags);

			m_world.SetAllowSleeping(settings.enableSleep > 0);
			m_world.SetWarmStarting(settings.enableWarmStarting > 0);
			m_world.SetContinuousPhysics(settings.enableContinuous > 0);
			m_world.SetSubStepping(settings.enableSubStepping > 0);

			m_pointCount = 0;

			m_world.Step(timeStep, settings.velocityIterations, settings.positionIterations);

			m_world.DrawDebugData();

			if (timeStep > 0.0f)
			{
				++m_stepCount;
			}

			if (settings.drawStats)
			{
				int bodyCount = m_world.GetBodyCount();
				int contactCount = m_world.GetContactCount();
				int jointCount = m_world.GetJointCount();
				m_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
				m_textLine += DRAW_STRING_NEW_LINE;

				int proxyCount = m_world.GetProxyCount();
				int height = m_world.GetTreeHeight();
				int balance = m_world.GetTreeBalance();
				float quality = m_world.GetTreeQuality();
				m_debugDraw.DrawString(5, m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
				m_textLine += DRAW_STRING_NEW_LINE;
			}

			// Track maximum profile times
			{
				const b2Profile& p = m_world.GetProfile();
				m_maxProfile.step = b2Max(m_maxProfile.step, p.step);
				m_maxProfile.collide = b2Max(m_maxProfile.collide, p.collide);
				m_maxProfile.solve = b2Max(m_maxProfile.solve, p.solve);
				m_maxProfile.solveInit = b2Max(m_maxProfile.solveInit, p.solveInit);
				m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
				m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
				m_maxProfile.solveTOI = b2Max(m_maxProfile.solveTOI, p.solveTOI);
				m_maxProfile.broadphase = b2Max(m_maxProfile.broadphase, p.broadphase);

				m_totalProfile.step += p.step;
				m_totalProfile.collide += p.collide;
				m_totalProfile.solve += p.solve;
				m_totalProfile.solveInit += p.solveInit;
				m_totalProfile.solveVelocity += p.solveVelocity;
				m_totalProfile.solvePosition += p.solvePosition;
				m_totalProfile.solveTOI += p.solveTOI;
				m_totalProfile.broadphase += p.broadphase;
			}

			if (settings.drawProfile)
			{
				const b2Profile& p = m_world.GetProfile();

				b2Profile aveProfile;
				memset(&aveProfile, 0, sizeof(b2Profile));
				if (m_stepCount > 0)
				{
					float scale = 1.0f / m_stepCount;
					aveProfile.step = scale * m_totalProfile.step;
					aveProfile.collide = scale * m_totalProfile.collide;
					aveProfile.solve = scale * m_totalProfile.solve;
					aveProfile.solveInit = scale * m_totalProfile.solveInit;
					aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
					aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
					aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
					aveProfile.broadphase = scale * m_totalProfile.broadphase;
				}

				m_debugDraw.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
				m_textLine += DRAW_STRING_NEW_LINE;
				m_debugDraw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
				m_textLine += DRAW_STRING_NEW_LINE;
			}

			if (m_mouseJoint)
			{
				b2Vec2 p1 = m_mouseJoint.GetAnchorB();
				b2Vec2 p2 = m_mouseJoint.GetTarget();

				b2Color c;
				c.Set(0.0f, 1.0f, 0.0f);
				m_debugDraw.DrawPoint(p1, 4.0f, c);
				m_debugDraw.DrawPoint(p2, 4.0f, c);

				c.Set(0.8f, 0.8f, 0.8f);
				m_debugDraw.DrawSegment(p1, p2, c);
			}
	
			if (m_bombSpawning)
			{
				b2Color c;
				c.Set(0.0f, 0.0f, 1.0f);
				m_debugDraw.DrawPoint(m_bombSpawnPoint, 4.0f, c);

				c.Set(0.8f, 0.8f, 0.8f);
				m_debugDraw.DrawSegment(m_mouseWorld, m_bombSpawnPoint, c);
			}

			if (settings.drawContactPoints)
			{
				const float k_impulseScale = 0.1f;
				const float k_axisScale = 0.3f;

				for (int i = 0; i < m_pointCount; ++i)
				{
					ContactPoint* point = m_points + i;

					if (point.state == b2_addState)
					{
						// Add
						m_debugDraw.DrawPoint(point.position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
					}
					else if (point.state == b2_persistState)
					{
						// Persist
						m_debugDraw.DrawPoint(point.position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
					}

					if (settings.drawContactNormals == 1)
					{
						b2Vec2 p1 = point.position;
						b2Vec2 p2 = p1 + k_axisScale * point.normal;
						m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
					}
					else if (settings.drawContactImpulse == 1)
					{
						b2Vec2 p1 = point.position;
						b2Vec2 p2 = p1 + k_impulseScale * point.normalImpulse * point.normal;
						m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
					}

					if (settings.drawFrictionImpulse == 1)
					{
						b2Vec2 tangent = Utilities.b2Cross(point.normal, 1.0f);
						b2Vec2 p1 = point.position;
						b2Vec2 p2 = p1 + k_impulseScale * point.tangentImpulse * tangent;
						m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
					}
				}
			}
		}
		public virtual void Keyboard(char key) {  
		
		}

		public abstract void KeyboardUp(char key) {
		
		}

		public void ShiftMouseDown(b2Vec2 p){
			m_mouseWorld = p;
	
			if (m_mouseJoint != null)
			{
				return;
			}

			SpawnBomb(p);
		}
		public virtual void MouseDown(b2Vec2 p){
			m_mouseWorld = p;
	
			if (m_mouseJoint != null)
			{
				return;
			}

			// Make a small box.
			b2AABB aabb;
			b2Vec2 d;
			d.Set(0.001f, 0.001f);
			aabb.lowerBound = p - d;
			aabb.upperBound = p + d;

			// Query the world for overlapping shapes.
			QueryCallback callback(p);
			m_world.QueryAABB(&callback, aabb);

			if (callback.m_fixture)
			{
				b2Body* body = callback.m_fixture.GetBody();
				b2MouseJointDef md;
				md.bodyA = m_groundBody;
				md.bodyB = body;
				md.target = p;
				md.maxForce = 1000.0f * body.GetMass();
				m_mouseJoint = (b2MouseJoint*)m_world.CreateJoint(&md);
				body.SetAwake(true);
			}
		}
		public abstract void MouseUp(b2Vec2 p){
			if (m_mouseJoint)
			{
				m_world.DestroyJoint(m_mouseJoint);
				m_mouseJoint = null;
			}
	
			if (m_bombSpawning)
			{
				CompleteBombSpawn(p);
			}
		}
		public void MouseMove(b2Vec2 p){
			m_mouseWorld = p;
	
			if (m_mouseJoint)
			{
				m_mouseJoint.SetTarget(p);
			}
		}
		public void LaunchBomb(){
			b2Vec2 p(RandomFloat(-15.0f, 15.0f), 30.0f);
			b2Vec2 v = -5.0f * p;
			LaunchBomb(p, v);
		}
		public void LaunchBomb(b2Vec2 position, b2Vec2 velocity){
			if (m_bomb)
			{
				m_world.DestroyBody(m_bomb);
				m_bomb = null;
			}

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = position;
			bd.bullet = true;
			m_bomb = m_world.CreateBody(&bd);
			m_bomb.SetLinearVelocity(velocity);
	
			b2CircleShape circle;
			circle.m_radius = 0.3f;

			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 20.0f;
			fd.restitution = 0.0f;
	
			b2Vec2 minV = position - b2Vec2(0.3f,0.3f);
			b2Vec2 maxV = position + b2Vec2(0.3f,0.3f);
	
			b2AABB aabb;
			aabb.lowerBound = minV;
			aabb.upperBound = maxV;

			m_bomb.CreateFixture(&fd);
		}
	
		public void SpawnBomb(b2Vec2 worldPt){
			m_bombSpawnPoint = worldPt;
			m_bombSpawning = true;
		}
		public void CompleteBombSpawn(b2Vec2 p){
			if (m_bombSpawning == false)
			{
				return;
			}

			const float multiplier = 30.0f;
			b2Vec2 vel = m_bombSpawnPoint - p;
			vel *= multiplier;
			LaunchBomb(m_bombSpawnPoint,vel);
			m_bombSpawning = false;
		}

		// Let derived tests know that a joint was destroyed.
		public virtual void JointDestroyed(b2Joint joint) {  }

		// Callbacks for derived classes.
		public abstract void BeginContact(b2Contact contact) {  }
		public abstract void EndContact(b2Contact contact) {  }
		public abstract void PreSolve(b2Contact contact, b2Manifold oldManifold){
			const b2Manifold* manifold = contact.GetManifold();

			if (manifold.pointCount == 0)
			{
				return;
			}

			b2Fixture* fixtureA = contact.GetFixtureA();
			b2Fixture* fixtureB = contact.GetFixtureB();

			b2PointState state1[b2Settings.b2_maxManifoldPoints], state2[b2Settings.b2_maxManifoldPoints];
			b2GetPointStates(state1, state2, oldManifold, manifold);

			b2WorldManifold worldManifold;
			contact.GetWorldManifold(&worldManifold);

			for (int i = 0; i < manifold.pointCount && m_pointCount < k_maxContactPoints; ++i)
			{
				ContactPoint* cp = m_points + m_pointCount;
				cp.fixtureA = fixtureA;
				cp.fixtureB = fixtureB;
				cp.position = worldManifold.points[i];
				cp.normal = worldManifold.normal;
				cp.state = state2[i];
				cp.normalImpulse = manifold.points[i].normalImpulse;
				cp.tangentImpulse = manifold.points[i].tangentImpulse;
				++m_pointCount;
			}
		}

		public abstract void PostSolve(b2Contact contact, b2ContactImpulse impulse)
		{
		}

		public void ShiftOrigin(b2Vec2 newOrigin){
			m_world.ShiftOrigin(newOrigin);
		}

		protected b2Body m_groundBody;
		protected b2AABB m_worldAABB;
		protected ContactPoint m_points[k_maxContactPoints];
		protected int m_pointCount;
		protected DestructionListener m_destructionListener;
		protected DebugDraw m_debugDraw;
		protected int m_textLine;
		protected b2World m_world;
		protected b2Body m_bomb;
		protected b2MouseJoint m_mouseJoint;
		protected b2Vec2 m_bombSpawnPoint;
		protected bool m_bombSpawning;
		protected b2Vec2 m_mouseWorld;
		protected int m_stepCount;

		protected b2Profile m_maxProfile;
		protected b2Profile m_totalProfile;
	}
}