using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;
using GLImp;
using System.Drawing;

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
			b2Vec2 gravity = new b2Vec2();
			gravity.Set(0.0f, -10.0f);
			m_world = new b2World(gravity);
			m_bomb = null;
			m_textLine = 30;
			m_mouseJoint = null;
			m_pointCount = 0;
			m_debugDraw = new DebugDraw();

			m_destructionListener = new DestructionListener();
			m_destructionListener.test = this;
			m_world.SetDestructionListener(m_destructionListener);
			m_world.SetContactListener(this);
			m_world.SetDebugDraw(m_debugDraw);
	
			m_bombSpawning = false;

			m_stepCount = 0;

			b2BodyDef bodyDef = new b2BodyDef();
			m_groundBody = m_world.CreateBody(bodyDef);

			m_maxProfile = new b2Profile();
			m_totalProfile = new b2Profile();
		}

		public void DrawTitle(string title){
			Console.Clear();
			m_debugDraw.DrawString(title);
			Console.WriteLine();
		}

		public virtual void Step(Settings settings) {
			float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

			if (settings.pause)
			{
				if (settings.singleStep)
				{
					settings.singleStep = false;
				}
				else
				{
					timeStep = 0.0f;
				}

				m_debugDraw.DrawString("****PAUSED****");
			}

			b2Draw.DrawFlags flags = 0;
			flags |= settings.drawShapes ? b2Draw.DrawFlags.e_shapeBit : 0;
			flags |= settings.drawJoints ? b2Draw.DrawFlags.e_jointBit : 0;
			flags |= settings.drawAABBs  ? b2Draw.DrawFlags.e_aabbBit  : 0;
			flags |= settings.drawCOMs   ? b2Draw.DrawFlags.e_centerOfMassBit : 0;
			m_debugDraw.SetFlags(flags);

			m_world.SetAllowSleeping(settings.enableSleep);
			m_world.SetWarmStarting(settings.enableWarmStarting);
			m_world.SetContinuousPhysics(settings.enableContinuous);
			m_world.SetSubStepping(settings.enableSubStepping);

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
				m_debugDraw.DrawString("bodies/contacts/joints = {0}/{1}/{2}", bodyCount, contactCount, jointCount);

				int proxyCount = m_world.GetProxyCount();
				int height = m_world.GetTreeHeight();
				int balance = m_world.GetTreeBalance();
				float quality = m_world.GetTreeQuality();
				m_debugDraw.DrawString("proxies/height/balance/quality = {0}/{1}/{2}/{3}", proxyCount, height, balance, quality);
			}

			// Track maximum profile times
			{
				b2Profile p = m_world.GetProfile();
				m_maxProfile.step = Math.Max(m_maxProfile.step, p.step);
				m_maxProfile.collide = Math.Max(m_maxProfile.collide, p.collide);
				m_maxProfile.solve = Math.Max(m_maxProfile.solve, p.solve);
				m_maxProfile.solveInit = Math.Max(m_maxProfile.solveInit, p.solveInit);
				m_maxProfile.solveVelocity = Math.Max(m_maxProfile.solveVelocity, p.solveVelocity);
				m_maxProfile.solvePosition = Math.Max(m_maxProfile.solvePosition, p.solvePosition);
				m_maxProfile.solveTOI = Math.Max(m_maxProfile.solveTOI, p.solveTOI);
				m_maxProfile.broadphase = Math.Max(m_maxProfile.broadphase, p.broadphase);

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
				b2Profile p = m_world.GetProfile();

				b2Profile aveProfile = new b2Profile();
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

				m_debugDraw.DrawString("step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
				m_debugDraw.DrawString("collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
				m_debugDraw.DrawString("solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
				m_debugDraw.DrawString("solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
				m_debugDraw.DrawString("solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
				m_debugDraw.DrawString("solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
				m_debugDraw.DrawString("solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
				m_debugDraw.DrawString("broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
			}

			if (m_mouseJoint != null)
			{
				b2Vec2 p1 = m_mouseJoint.GetAnchorB();
				b2Vec2 p2 = m_mouseJoint.GetTarget();

				Color c = Color.FromArgb(0, 255, 0);
				m_debugDraw.DrawPoint(p1, 4.0f, c);
				m_debugDraw.DrawPoint(p2, 4.0f, c);

				c = Color.FromArgb(204, 204, 204);
				m_debugDraw.DrawSegment(p1, p2, c);
			}
	
			if (m_bombSpawning)
			{
				Color c = Color.FromArgb(0, 0, 255);
				m_debugDraw.DrawPoint(m_bombSpawnPoint, 4.0f, c);

				c = Color.FromArgb(200, 200, 200);
				m_debugDraw.DrawSegment(m_mouseWorld, m_bombSpawnPoint, c);
			}

			if (settings.drawContactPoints)
			{
				const float k_impulseScale = 0.1f;
				const float k_axisScale = 0.3f;

				for (int i = 0; i < m_pointCount; ++i)
				{
					ContactPoint point = m_points[i];

					if (point.state == b2PointState.b2_addState)
					{
						// Add
						m_debugDraw.DrawPoint(point.position, 10.0f, Color.FromArgb(75, 242, 75));
					}
					else if (point.state == b2PointState.b2_persistState)
					{
						// Persist
						m_debugDraw.DrawPoint(point.position, 5.0f, Color.FromArgb(75, 75, 242));
					}

					if (settings.drawContactNormals == 1)
					{
						b2Vec2 p1 = point.position;
						b2Vec2 p2 = p1 + k_axisScale * point.normal;
						m_debugDraw.DrawSegment(p1, p2, Color.FromArgb(242, 242, 242));
					}
					else if (settings.drawContactImpulse == 1)
					{
						b2Vec2 p1 = point.position;
						b2Vec2 p2 = p1 + k_impulseScale * point.normalImpulse * point.normal;
						m_debugDraw.DrawSegment(p1, p2, Color.FromArgb(242, 242, 75));
					}

					if (settings.drawFrictionImpulse == 1)
					{
						b2Vec2 tangent = Utilities.b2Cross(point.normal, 1.0f);
						b2Vec2 p1 = point.position;
						b2Vec2 p2 = p1 + k_impulseScale * point.tangentImpulse * tangent;
						m_debugDraw.DrawSegment(p1, p2, Color.FromArgb(242, 242, 75));
					}
				}
			}
		}
		public virtual void Keyboard() {  
		
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
			b2Vec2 d = new b2Vec2();
			d.Set(0.001f, 0.001f);
			aabb.lowerBound = p - d;
			aabb.upperBound = p + d;

			// Query the world for overlapping shapes.
			QueryCallback callback = new QueryCallback(p);
			m_world.QueryAABB(callback, aabb);

			if (callback.m_fixture != null)
			{
				b2Body body = callback.m_fixture.GetBody();
				b2MouseJointDef md = new b2MouseJointDef();
				md.bodyA = m_groundBody;
				md.bodyB = body;
				md.target = p;
				md.maxForce = 1000.0f * body.GetMass();
				m_mouseJoint = (b2MouseJoint)m_world.CreateJoint(md);
				body.SetAwake(true);
			}
		}
		public virtual void MouseUp(b2Vec2 p) {
			if (m_mouseJoint != null)
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
	
			if (m_mouseJoint != null)
			{
				m_mouseJoint.SetTarget(p);
			}
		}
		public void LaunchBomb(){
			b2Vec2 p = new b2Vec2(RandomFloat(-15.0f, 15.0f), 30.0f);
			b2Vec2 v = -5.0f * p;
			LaunchBomb(p, v);
		}
		public void LaunchBomb(b2Vec2 position, b2Vec2 velocity){
			if (m_bomb != null)
			{
				m_world.DestroyBody(m_bomb);
				m_bomb = null;
			}

			b2BodyDef bd = new b2BodyDef();
			bd.type = b2BodyType.b2_dynamicBody;
			bd.position = position;
			bd.bullet = true;
			m_bomb = m_world.CreateBody(bd);
			m_bomb.SetLinearVelocity(velocity);
	
			b2CircleShape circle = new b2CircleShape();
			circle.m_radius = 0.3f;

			b2FixtureDef fd = new b2FixtureDef();
			fd.shape = circle;
			fd.density = 20.0f;
			fd.restitution = 0.0f;
	
			b2Vec2 minV = position - new b2Vec2(0.3f,0.3f);
			b2Vec2 maxV = position + new b2Vec2(0.3f, 0.3f);
	
			b2AABB aabb;
			aabb.lowerBound = minV;
			aabb.upperBound = maxV;

			m_bomb.CreateFixture(fd);
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
		public virtual void BeginContact(b2Contact contact) { }
		public virtual void EndContact(b2Contact contact) { }
		public virtual void PreSolve(b2Contact contact, b2Manifold oldManifold) {
			b2Manifold manifold = contact.GetManifold();

			if (manifold.points.Count() == 0)
			{
				return;
			}

			b2Fixture fixtureA = contact.GetFixtureA();
			b2Fixture fixtureB = contact.GetFixtureB();

			b2PointState[] state1 = new b2PointState[b2Settings.b2_maxManifoldPoints];
			b2PointState[] state2 = new b2PointState[b2Settings.b2_maxManifoldPoints];
			b2Collision.b2GetPointStates(state1, state2, oldManifold, manifold);

			b2WorldManifold worldManifold;
			contact.GetWorldManifold(out worldManifold);

			for (int i = 0; i < manifold.points.Count() && m_pointCount < Program.k_maxContactPoints; ++i)
			{
				ContactPoint cp = m_points[m_pointCount];
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

		public virtual void PostSolve(b2Contact contact, b2ContactImpulse impulse)
		{
		}

		public void ShiftOrigin(b2Vec2 newOrigin){
			m_world.ShiftOrigin(newOrigin);
		}

		protected b2Body m_groundBody;
		protected b2AABB m_worldAABB;
		protected ContactPoint[] m_points = new ContactPoint[Program.k_maxContactPoints];
		protected int m_pointCount;
		protected DestructionListener m_destructionListener;
		protected DebugDraw m_debugDraw;
		protected int m_textLine;
		internal b2World m_world;
		protected b2Body m_bomb;
		public b2MouseJoint m_mouseJoint;
		protected b2Vec2 m_bombSpawnPoint;
		protected bool m_bombSpawning;
		protected b2Vec2 m_mouseWorld;
		protected int m_stepCount;

		protected b2Profile m_maxProfile;
		protected b2Profile m_totalProfile;
	}
}
