using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;
using GLImp;
using System.Drawing;

namespace Testbed.Framework {
	abstract class Test : ContactListener
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
			Vec2 gravity = new Vec2();
			gravity.Set(0.0f, -10.0f);
			m_world = new World(gravity);
			m_bomb = null;
			m_textLine = 30;
			m_mouseJoint = null;
			m_pointCount = 0;
			m_debugDraw = new DebugDraw();

			m_destructionListener = new TestDestructionListener();
			m_destructionListener.test = this;
			m_world.SetDestructionListener(m_destructionListener);
			m_world.SetContactListener(this);
			m_world.SetDebugDraw(m_debugDraw);
	
			m_bombSpawning = false;

			m_stepCount = 0;

			BodyDef bodyDef = new BodyDef();
			m_groundBody = m_world.CreateBody(bodyDef);

			m_maxProfile = new Profile();
			m_totalProfile = new Profile();
		}

		public void DrawTitle(string title){
			Console.Clear();
			m_debugDraw.DrawString(title);
			Console.WriteLine();
		}

		public virtual void Step(TestSettings settings) {
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

			Draw.DrawFlags flags = 0;
			flags |= settings.drawShapes ? Draw.DrawFlags.e_shapeBit : 0;
			flags |= settings.drawJoints ? Draw.DrawFlags.e_jointBit : 0;
			flags |= settings.drawAABBs  ? Draw.DrawFlags.e_aabbBit  : 0;
			flags |= settings.drawCOMs   ? Draw.DrawFlags.e_centerOfMassBit : 0;
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
				Profile p = m_world.GetProfile();
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
				Profile p = m_world.GetProfile();

				Profile aveProfile = new Profile();
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
				Vec2 p1 = m_mouseJoint.GetAnchorB();
				Vec2 p2 = m_mouseJoint.GetTarget();

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

					if (point.state == PointState._addState)
					{
						// Add
						m_debugDraw.DrawPoint(point.position, 10.0f, Color.FromArgb(75, 242, 75));
					}
					else if (point.state == PointState._persistState)
					{
						// Persist
						m_debugDraw.DrawPoint(point.position, 5.0f, Color.FromArgb(75, 75, 242));
					}

					if (settings.drawContactNormals)
					{
						Vec2 p1 = point.position;
						Vec2 p2 = p1 + k_axisScale * point.normal;
						m_debugDraw.DrawSegment(p1, p2, Color.FromArgb(242, 242, 242));
					}
					else if (settings.drawContactImpulse)
					{
						Vec2 p1 = point.position;
						Vec2 p2 = p1 + k_impulseScale * point.normalImpulse * point.normal;
						m_debugDraw.DrawSegment(p1, p2, Color.FromArgb(242, 242, 75));
					}

					if (settings.drawFrictionImpulse)
					{
						Vec2 tangent = Utilities.Cross(point.normal, 1.0f);
						Vec2 p1 = point.position;
						Vec2 p2 = p1 + k_impulseScale * point.tangentImpulse * tangent;
						m_debugDraw.DrawSegment(p1, p2, Color.FromArgb(242, 242, 75));
					}
				}
			}
		}
		public virtual void Keyboard() {  
		
		}

		public void ShiftMouseDown(Vec2 p){
			m_mouseWorld = p;
	
			if (m_mouseJoint != null)
			{
				return;
			}

			SpawnBomb(p);
		}
		public virtual void MouseDown(Vec2 p){
			m_mouseWorld = p;
	
			if (m_mouseJoint != null)
			{
				return;
			}

			// Make a small box.
			AABB aabb;
			Vec2 d = new Vec2();
			d.Set(0.001f, 0.001f);
			aabb.lowerBound = p - d;
			aabb.upperBound = p + d;

			// Query the world for overlapping shapes.
			TestQueryCallback callback = new TestQueryCallback(p);
			m_world.QueryAABB(callback, aabb);

			if (callback.m_fixture != null)
			{
				Body body = callback.m_fixture.GetBody();
				MouseJointDef md = new MouseJointDef();
				md.bodyA = m_groundBody;
				md.bodyB = body;
				md.target = p;
				md.maxForce = 1000.0f * body.GetMass();
				m_mouseJoint = (MouseJoint)m_world.CreateJoint(md);
				body.SetAwake(true);
			}
		}
		public virtual void MouseUp(Vec2 p) {
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
		public void MouseMove(Vec2 p){
			m_mouseWorld = p;
	
			if (m_mouseJoint != null)
			{
				m_mouseJoint.SetTarget(p);
			}
		}
		public void LaunchBomb(){
			Vec2 p = new Vec2(RandomFloat(-15.0f, 15.0f), 30.0f);
			Vec2 v = -5.0f * p;
			LaunchBomb(p, v);
		}
		public void LaunchBomb(Vec2 position, Vec2 velocity){
			if (m_bomb != null)
			{
				m_world.DestroyBody(m_bomb);
				m_bomb = null;
			}

			BodyDef bd = new BodyDef();
			bd.type = BodyType._dynamicBody;
			bd.Position = position;
			bd.bullet = true;
			m_bomb = m_world.CreateBody(bd);
			m_bomb.SetLinearVelocity(velocity);
	
			CircleShape circle = new CircleShape();
			circle.m_radius = 0.3f;

			FixtureDef fd = new FixtureDef();
			fd.shape = circle;
			fd.Density = 20.0f;
			fd.Restitution = 0.0f;
	
			Vec2 minV = position - new Vec2(0.3f,0.3f);
			Vec2 maxV = position + new Vec2(0.3f, 0.3f);
	
			AABB aabb;
			aabb.lowerBound = minV;
			aabb.upperBound = maxV;

			m_bomb.CreateFixture(fd);
		}
	
		public void SpawnBomb(Vec2 worldPt){
			m_bombSpawnPoint = worldPt;
			m_bombSpawning = true;
		}
		public void CompleteBombSpawn(Vec2 p){
			if (m_bombSpawning == false)
			{
				return;
			}

			const float multiplier = 30.0f;
			Vec2 vel = m_bombSpawnPoint - p;
			vel *= multiplier;
			LaunchBomb(m_bombSpawnPoint,vel);
			m_bombSpawning = false;
		}

		// Let derived tests know that a joint was destroyed.
		public virtual void JointDestroyed(Joint joint) {  }

		// Callbacks for derived classes.
		public virtual void BeginContact(Contact contact) { }
		public virtual void EndContact(Contact contact) { }
		public virtual void PreSolve(Contact contact, Manifold oldManifold) {
			Manifold manifold = contact.GetManifold();

			if (manifold.points.Count() == 0)
			{
				return;
			}

			Fixture fixtureA = contact.FixtureA;
			Fixture fixtureB = contact.FixtureB;

			PointState[] state1 = new PointState[Settings._maxManifoldPoints];
			PointState[] state2 = new PointState[Settings._maxManifoldPoints];
			Collision.GetPointStates(state1, state2, oldManifold, manifold);

			WorldManifold worldManifold;
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

		public virtual void PostSolve(Contact contact, ContactImpulse impulse)
		{
		}

		public void ShiftOrigin(Vec2 newOrigin){
			m_world.ShiftOrigin(newOrigin);
		}

		protected Body m_groundBody;
		protected AABB m_worldAABB;
		protected ContactPoint[] m_points = new ContactPoint[Program.k_maxContactPoints];
		protected int m_pointCount;
		protected TestDestructionListener m_destructionListener;
		protected DebugDraw m_debugDraw;
		protected int m_textLine;
		internal World m_world;
		protected Body m_bomb;
		public MouseJoint m_mouseJoint;
		protected Vec2 m_bombSpawnPoint;
		protected bool m_bombSpawning;
		protected Vec2 m_mouseWorld;
		protected int m_stepCount;

		protected Profile m_maxProfile;
		protected Profile m_totalProfile;
	}
}
