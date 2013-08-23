﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Diagnostics;

namespace Box2D {
	// m_flags
	internal enum WorldFlags {
		e_newFixture = 0x0001,
		e_locked = 0x0002,
		e_clearForces = 0x0004
	}

	/// The world class manages all physics entities, dynamic simulation,
	/// and asynchronous queries. The world also contains efficient memory
	/// management facilities.
	public class b2World {
		/// Construct a world object.
		/// @param gravity the world gravity vector.
		public b2World(b2Vec2 gravity){
			m_destructionListener = null;
			m_debugDraw = null;

			m_bodyList = new List<b2Body>();
			m_jointList = new List<b2Joint>();

			m_jointCount = 0;

			m_warmStarting = true;
			m_continuousPhysics = true;
			m_subStepping = false;

			m_stepComplete = true;

			m_allowSleep = true;
			m_gravity = gravity;

			m_flags = WorldFlags.e_clearForces;

			m_inv_dt0 = 0.0f;

			m_profile = new b2Profile();
		}

		/// Destruct the world. All physics entities are destroyed and all heap memory is released.
		~b2World(){
			throw new NotImplementedException();
			//// Some shapes allocate using b2Alloc.
			//b2Body* b = m_bodyList;
			//while (b)
			//{
			//    b2Body* bNext = b.m_next;

			//    b2Fixture* f = b.m_fixtureList;
			//    while (f)
			//    {
			//        b2Fixture* fNext = f.m_next;
			//        f.m_proxyCount = 0;
			//        f.Destroy(&m_blockAllocator);
			//        f = fNext;
			//    }

			//    b = bNext;
			//}
		}

		/// Register a destruction listener. The listener is owned by you and must
		/// remain in scope.
		public void SetDestructionListener(b2DestructionListener listener){
			throw new NotImplementedException();
			//m_destructionListener = listener;
		}

		/// Register a contact filter to provide specific control over collision.
		/// Otherwise the default filter is used (b2_defaultFilter). The listener is
		/// owned by you and must remain in scope. 
		public void SetContactFilter(b2ContactFilter filter){
			throw new NotImplementedException();
			//m_contactManager.m_contactFilter = filter;
		}

		/// Register a contact event listener. The listener is owned by you and must
		/// remain in scope.
		public void SetContactListener(b2ContactListener listener){
			throw new NotImplementedException();
			//m_contactManager.m_contactListener = listener;
		}

		/// Register a routine for debug drawing. The debug draw functions are called
		/// inside with b2World::DrawDebugData method. The debug draw object is owned
		/// by you and must remain in scope.
		public void SetDebugDraw(b2Draw debugDraw){
			throw new NotImplementedException();
			//m_debugDraw = debugDraw;
		}

		/// Create a rigid body given a definition. No reference to the definition
		/// is retained.
		/// @warning This function is locked during callbacks.
		public b2Body CreateBody(b2BodyDef def){
			Utilities.Assert(IsLocked() == false);
			if (IsLocked())
			{
			    return null;
			}
			
			b2Body b = new b2Body(def, this);

			// Add to world doubly linked list.
			m_bodyList.Add(b);

			return b;
		}

		/// Destroy a rigid body given a definition. No reference to the definition
		/// is retained. This function is locked during callbacks.
		/// @warning This automatically deletes all associated shapes and joints.
		/// @warning This function is locked during callbacks.
		public void DestroyBody(b2Body body){
			throw new NotImplementedException();
			//Utilities.Assert(m_bodyCount > 0);
			//Utilities.Assert(IsLocked() == false);
			//if (IsLocked())
			//{
			//    return;
			//}

			//// Delete the attached joints.
			//b2JointEdge* je = b.m_jointList;
			//while (je)
			//{
			//    b2JointEdge* je0 = je;
			//    je = je.next;

			//    if (m_destructionListener)
			//    {
			//        m_destructionListener.SayGoodbye(je0.joint);
			//    }

			//    DestroyJoint(je0.joint);

			//    b.m_jointList = je;
			//}
			//b.m_jointList = null;

			//// Delete the attached contacts.
			//b2ContactEdge* ce = b.m_contactList;
			//while (ce)
			//{
			//    b2ContactEdge* ce0 = ce;
			//    ce = ce.next;
			//    m_contactManager.Destroy(ce0.contact);
			//}
			//b.m_contactList = null;

			//// Delete the attached fixtures. This destroys broad-phase proxies.
			//b2Fixture* f = b.m_fixtureList;
			//while (f)
			//{
			//    b2Fixture* f0 = f;
			//    f = f.m_next;

			//    if (m_destructionListener)
			//    {
			//        m_destructionListener.SayGoodbye(f0);
			//    }

			//    f0.DestroyProxies(&m_contactManager.m_broadPhase);
			//    f0.Destroy(&m_blockAllocator);
			//    f0.~b2Fixture();
			//    m_blockAllocator.Free(f0, sizeof(b2Fixture));

			//    b.m_fixtureList = f;
			//    b.m_fixtureCount -= 1;
			//}
			//b.m_fixtureList = null;
			//b.m_fixtureCount = 0;

			//// Remove world body list.
			//if (b.m_prev)
			//{
			//    b.m_prev.m_next = b.m_next;
			//}

			//if (b.m_next)
			//{
			//    b.m_next.m_prev = b.m_prev;
			//}

			//if (b == m_bodyList)
			//{
			//    m_bodyList = b.m_next;
			//}

			//--m_bodyCount;
			//b.~b2Body();
			//m_blockAllocator.Free(b, sizeof(b2Body));
		}

		/// Create a joint to constrain bodies together. No reference to the definition
		/// is retained. This may cause the connected bodies to cease colliding.
		/// @warning This function is locked during callbacks.
		public b2Joint CreateJoint(b2JointDef def){
			throw new NotImplementedException();
			//Utilities.Assert(IsLocked() == false);
			//if (IsLocked())
			//{
			//    return null;
			//}

			//b2Joint* j = b2Joint::Create(def, &m_blockAllocator);

			//// Connect to the world list.
			//j.m_prev = null;
			//j.m_next = m_jointList;
			//if (m_jointList)
			//{
			//    m_jointList.m_prev = j;
			//}
			//m_jointList = j;
			//++m_jointCount;

			//// Connect to the bodies' doubly linked lists.
			//j.m_edgeA.joint = j;
			//j.m_edgeA.other = j.m_bodyB;
			//j.m_edgeA.prev = null;
			//j.m_edgeA.next = j.m_bodyA.m_jointList;
			//if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = &j.m_edgeA;
			//j.m_bodyA.m_jointList = &j.m_edgeA;

			//j.m_edgeB.joint = j;
			//j.m_edgeB.other = j.m_bodyA;
			//j.m_edgeB.prev = null;
			//j.m_edgeB.next = j.m_bodyB.m_jointList;
			//if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = &j.m_edgeB;
			//j.m_bodyB.m_jointList = &j.m_edgeB;

			//b2Body* bodyA = def.bodyA;
			//b2Body* bodyB = def.bodyB;

			//// If the joint prevents collisions, then flag any contacts for filtering.
			//if (def.collideConnected == false)
			//{
			//    b2ContactEdge* edge = bodyB.GetContactList();
			//    while (edge)
			//    {
			//        if (edge.other == bodyA)
			//        {
			//            // Flag the contact for filtering at the next time step (where either
			//            // body is awake).
			//            edge.contact.FlagForFiltering();
			//        }

			//        edge = edge.next;
			//    }
			//}

			//// Note: creating a joint doesn't wake the bodies.

			//return j;
		}

		/// Destroy a joint. This may cause the connected bodies to begin colliding.
		/// @warning This function is locked during callbacks.
		public void DestroyJoint(b2Joint joint){
			throw new NotImplementedException();
			//Utilities.Assert(IsLocked() == false);
			//if (IsLocked())
			//{
			//    return;
			//}

			//bool collideConnected = j.m_collideConnected;

			//// Remove from the doubly linked list.
			//if (j.m_prev)
			//{
			//    j.m_prev.m_next = j.m_next;
			//}

			//if (j.m_next)
			//{
			//    j.m_next.m_prev = j.m_prev;
			//}

			//if (j == m_jointList)
			//{
			//    m_jointList = j.m_next;
			//}

			//// Disconnect from island graph.
			//b2Body* bodyA = j.m_bodyA;
			//b2Body* bodyB = j.m_bodyB;

			//// Wake up connected bodies.
			//bodyA.SetAwake(true);
			//bodyB.SetAwake(true);

			//// Remove from body 1.
			//if (j.m_edgeA.prev)
			//{
			//    j.m_edgeA.prev.next = j.m_edgeA.next;
			//}

			//if (j.m_edgeA.next)
			//{
			//    j.m_edgeA.next.prev = j.m_edgeA.prev;
			//}

			//if (&j.m_edgeA == bodyA.m_jointList)
			//{
			//    bodyA.m_jointList = j.m_edgeA.next;
			//}

			//j.m_edgeA.prev = null;
			//j.m_edgeA.next = null;

			//// Remove from body 2
			//if (j.m_edgeB.prev)
			//{
			//    j.m_edgeB.prev.next = j.m_edgeB.next;
			//}

			//if (j.m_edgeB.next)
			//{
			//    j.m_edgeB.next.prev = j.m_edgeB.prev;
			//}

			//if (&j.m_edgeB == bodyB.m_jointList)
			//{
			//    bodyB.m_jointList = j.m_edgeB.next;
			//}

			//j.m_edgeB.prev = null;
			//j.m_edgeB.next = null;

			//b2Joint::Destroy(j, &m_blockAllocator);

			//Utilities.Assert(m_jointCount > 0);
			//--m_jointCount;

			//// If the joint prevents collisions, then flag any contacts for filtering.
			//if (collideConnected == false)
			//{
			//    b2ContactEdge* edge = bodyB.GetContactList();
			//    while (edge)
			//    {
			//        if (edge.other == bodyA)
			//        {
			//            // Flag the contact for filtering at the next time step (where either
			//            // body is awake).
			//            edge.contact.FlagForFiltering();
			//        }

			//        edge = edge.next;
			//    }
			//}
		}

		/// Take a time step. This performs collision detection, integration,
		/// and constraint solution.
		/// @param timeStep the amount of time to simulate, this should not vary.
		/// @param velocityIterations for the velocity constraint solver.
		/// @param positionIterations for the position constraint solver.
		public void Step(float timeStep, int velocityIterations, int positionIterations){
			Stopwatch stepTimer;

			// If new fixtures were added, we need to find the new contacts.
			if (m_flags.HasFlag(WorldFlags.e_newFixture)) {
				m_contactManager.FindNewContacts();
				m_flags &= ~WorldFlags.e_newFixture;
			}

			m_flags |= WorldFlags.e_locked;

			b2TimeStep step;
			step.dt = timeStep;
			step.velocityIterations = velocityIterations;
			step.positionIterations = positionIterations;
			if (timeStep > 0.0f) {
				step.inv_dt = 1.0f / timeStep;
			} else {
				step.inv_dt = 0.0f;
			}

			step.dtRatio = m_inv_dt0 * timeStep;

			step.warmStarting = m_warmStarting;

			// Update contacts. This is where some contacts are destroyed.
			{
				b2Timer timer;
				m_contactManager.Collide();
				m_profile.collide = timer.GetMilliseconds();
			}

			// Integrate velocities, solve velocity constraints, and integrate positions.
			if (m_stepComplete && step.dt > 0.0f) {
				b2Timer timer;
				Solve(step);
				m_profile.solve = timer.GetMilliseconds();
			}

			// Handle TOI events.
			if (m_continuousPhysics && step.dt > 0.0f) {
				b2Timer timer;
				SolveTOI(step);
				m_profile.solveTOI = timer.GetMilliseconds();
			}

			if (step.dt > 0.0f) {
				m_inv_dt0 = step.inv_dt;
			}

			if (m_flags & e_clearForces) {
				ClearForces();
			}

			m_flags &= ~e_locked;

			m_profile.step = stepTimer.GetMilliseconds();
		}

		/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
		/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
		/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
		/// a fixed sized time step under a variable frame-rate.
		/// When you perform sub-stepping you will disable auto clearing of forces and instead call
		/// ClearForces after all sub-steps are complete in one pass of your game loop.
		/// @see SetAutoClearForces
		public void ClearForces(){
			throw new NotImplementedException();
			//for (b2Body* body = m_bodyList; body; body = body.GetNext())
			//{
			//    body.m_force.SetZero();
			//    body.m_torque = 0.0f;
			//}
		}

		/// Call this to draw shapes and other debug draw data.
		public void DrawDebugData(){
			throw new NotImplementedException();
			//if (m_debugDraw == null)
			//{
			//    return;
			//}

			//uint flags = m_debugDraw.GetFlags();

			//if (flags & b2Draw::e_shapeBit)
			//{
			//    for (b2Body* b = m_bodyList; b; b = b.GetNext())
			//    {
			//        const b2Transform& xf = b.GetTransform();
			//        for (b2Fixture* f = b.GetFixtureList(); f; f = f.GetNext())
			//        {
			//            if (b.IsActive() == false)
			//            {
			//                DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
			//            }
			//            else if (b.GetType() == b2_staticBody)
			//            {
			//                DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
			//            }
			//            else if (b.GetType() == b2_kinematicBody)
			//            {
			//                DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
			//            }
			//            else if (b.IsAwake() == false)
			//            {
			//                DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
			//            }
			//            else
			//            {
			//                DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
			//            }
			//        }
			//    }
			//}

			//if (flags & b2Draw::e_jointBit)
			//{
			//    for (b2Joint* j = m_jointList; j; j = j.GetNext())
			//    {
			//        DrawJoint(j);
			//    }
			//}

			//if (flags & b2Draw::e_pairBit)
			//{
			//    b2Color color(0.3f, 0.9f, 0.9f);
			//    for (b2Contact* c = m_contactManager.m_contactList; c; c = c.GetNext())
			//    {
			//        //b2Fixture* fixtureA = c.GetFixtureA();
			//        //b2Fixture* fixtureB = c.GetFixtureB();

			//        //b2Vec2 cA = fixtureA.GetAABB().GetCenter();
			//        //b2Vec2 cB = fixtureB.GetAABB().GetCenter();

			//        //m_debugDraw.DrawSegment(cA, cB, color);
			//    }
			//}

			//if (flags & b2Draw::e_aabbBit)
			//{
			//    b2Color color(0.9f, 0.3f, 0.9f);
			//    b2BroadPhase* bp = &m_contactManager.m_broadPhase;

			//    for (b2Body* b = m_bodyList; b; b = b.GetNext())
			//    {
			//        if (b.IsActive() == false)
			//        {
			//            continue;
			//        }

			//        for (b2Fixture* f = b.GetFixtureList(); f; f = f.GetNext())
			//        {
			//            for (int i = 0; i < f.m_proxyCount; ++i)
			//            {
			//                b2FixtureProxy* proxy = f.m_proxies + i;
			//                b2AABB aabb = bp.GetFatAABB(proxy.proxyId);
			//                b2Vec2 vs[4];
			//                vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
			//                vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
			//                vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
			//                vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

			//                m_debugDraw.DrawPolygon(vs, 4, color);
			//            }
			//        }
			//    }
			//}

			//if (flags & b2Draw::e_centerOfMassBit)
			//{
			//    for (b2Body* b = m_bodyList; b; b = b.GetNext())
			//    {
			//        b2Transform xf = b.GetTransform();
			//        xf.p = b.GetWorldCenter();
			//        m_debugDraw.DrawTransform(xf);
			//    }
			//}
		}

		/// Query the world for all fixtures that potentially overlap the
		/// provided AABB.
		/// @param callback a user implemented callback class.
		/// @param aabb the query box.
		public void QueryAABB(b2QueryCallback callback, b2AABB aabb){
			throw new NotImplementedException();
			//b2WorldQueryWrapper wrapper;
			//wrapper.broadPhase = &m_contactManager.m_broadPhase;
			//wrapper.callback = callback;
			//m_contactManager.m_broadPhase.Query(&wrapper, aabb);
		}

		/// Ray-cast the world for all fixtures in the path of the ray. Your callback
		/// controls whether you get the closest point, any point, or n-points.
		/// The ray-cast ignores shapes that contain the starting point.
		/// @param callback a user implemented callback class.
		/// @param point1 the ray starting point
		/// @param point2 the ray ending point
		public void RayCast(b2RayCastCallback callback, b2Vec2 point1, b2Vec2 point2){
			throw new NotImplementedException();
			//b2WorldRayCastWrapper wrapper;
			//wrapper.broadPhase = &m_contactManager.m_broadPhase;
			//wrapper.callback = callback;
			//b2RayCastInput input;
			//input.maxFraction = 1.0f;
			//input.p1 = point1;
			//input.p2 = point2;
			//m_contactManager.m_broadPhase.RayCast(&wrapper, input);
		}

		/// Get the world body list. With the returned body, use b2Body::GetNext to get
		/// the next body in the world list. A null body indicates the end of the list.
		/// @return the head of the world body list.
		public b2Body GetBodyList(){
			throw new NotImplementedException();
			//return m_bodyList;
		}

		/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
		/// the next joint in the world list. A null joint indicates the end of the list.
		/// @return the head of the world joint list.
		public List<b2Joint> GetJointList(){
			throw new NotImplementedException();
			//return m_jointList;
		}

		/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
		/// the next contact in the world list. A null contact indicates the end of the list.
		/// @return the head of the world contact list.
		/// @warning contacts are created and destroyed in the middle of a time step.
		/// Use b2ContactListener to avoid missing contacts.
		public List<b2Contact> GetContactList(){
			throw new NotImplementedException();
			//return m_contactManager.m_contactList;
		}

		/// Enable/disable sleep.
		public void SetAllowSleeping(bool flag){
			throw new NotImplementedException();
			//if (flag == m_allowSleep)
			//{
			//    return;
			//}

			//m_allowSleep = flag;
			//if (m_allowSleep == false)
			//{
			//    for (b2Body* b = m_bodyList; b; b = b.m_next)
			//    {
			//        b.SetAwake(true);
			//    }
			//}
		}
		public bool GetAllowSleeping() { return m_allowSleep; }

		/// Enable/disable warm starting. For testing.
		public void SetWarmStarting(bool flag) { m_warmStarting = flag; }
		public bool GetWarmStarting() { return m_warmStarting; }

		/// Enable/disable continuous physics. For testing.
		public void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
		public bool GetContinuousPhysics() { return m_continuousPhysics; }

		/// Enable/disable single stepped continuous physics. For testing.
		public void SetSubStepping(bool flag) { m_subStepping = flag; }
		public bool GetSubStepping() { return m_subStepping; }

		/// Get the number of broad-phase proxies.
		public int GetProxyCount(){
			return m_contactManager.m_broadPhase.GetProxyCount();
		}

		/// Get the number of bodies.
		public int GetBodyCount(){
			return m_bodyList.Count();
		}

		/// Get the number of joints.
		public int GetJointCount(){
			return m_jointCount;
		}

		/// Get the number of contacts (each may have 0 or more contact points).
		public int GetContactCount(){
			return m_contactManager.m_contactCount;
		}

		/// Get the height of the dynamic tree.
		public int GetTreeHeight(){
			return m_contactManager.m_broadPhase.GetTreeHeight();
		}

		/// Get the balance of the dynamic tree.
		public int GetTreeBalance(){
			return m_contactManager.m_broadPhase.GetTreeBalance();
		}

		/// Get the quality metric of the dynamic tree. The smaller the better.
		/// The minimum is 1.
		public float GetTreeQuality() {
			return m_contactManager.m_broadPhase.GetTreeQuality();
		}


		/// Change the global gravity vector.
		public void SetGravity(b2Vec2 gravity){
			m_gravity = gravity;
		}
	
		/// Get the global gravity vector.
		public b2Vec2 GetGravity(){
			return m_gravity;
		}

		/// Is the world locked (in the middle of a time step).
		public bool IsLocked(){
			return (m_flags & WorldFlags.e_locked) == WorldFlags.e_locked;
		}

		/// Set flag to control automatic clearing of forces after each time step.
		public void SetAutoClearForces(bool flag){
			if (flag)
			{
				m_flags |= WorldFlags.e_clearForces;
			}
			else
			{
				m_flags &= ~WorldFlags.e_clearForces;
			}
		}

		/// Get the flag that controls automatic clearing of forces after each time step.
		public bool GetAutoClearForces(){
			return (m_flags & WorldFlags.e_clearForces) == WorldFlags.e_clearForces;
		}

		/// Shift the world origin. Useful for large worlds.
		/// The body shift formula is: position -= newOrigin
		/// @param newOrigin the new origin with respect to the old origin
		public void ShiftOrigin(b2Vec2 newOrigin){
			throw new NotImplementedException();
			//Utilities.Assert((m_flags & WorldFlags.e_locked) == 0);
			//if ((m_flags & e_locked) == e_locked)
			//{
			//    return;
			//}

			//for (b2Body* b = m_bodyList; b; b = b.m_next)
			//{
			//    b.m_xf.p -= newOrigin;
			//    b.m_sweep.c0 -= newOrigin;
			//    b.m_sweep.c -= newOrigin;
			//}

			//for (b2Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    j.ShiftOrigin(newOrigin);
			//}

			//m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
		}

		/// Get the contact manager for testing.
		public b2ContactManager GetContactManager(){
			return m_contactManager;
		}

		/// Get the current profile.
		public b2Profile GetProfile(){
			return m_profile;
		}

		/// Dump the world into the log file.
		/// @warning this should be called outside of a time step.
		public void Dump(){
			throw new NotImplementedException();
			//if ((m_flags & e_locked) == e_locked)
			//{
			//    return;
			//}

			//b2Settings.b2Log("b2Vec2 g(%.15lef, %.15lef);\n", m_gravity.x, m_gravity.y);
			//b2Settings.b2Log("m_world.SetGravity(g);\n");

			//b2Settings.b2Log("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
			//b2Settings.b2Log("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);
			//int i = 0;
			//for (b2Body* b = m_bodyList; b; b = b.m_next)
			//{
			//    b.m_islandIndex = i;
			//    b.Dump();
			//    ++i;
			//}

			//i = 0;
			//for (b2Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    j.m_index = i;
			//    ++i;
			//}

			//// First pass on joints, skip gear joints.
			//for (b2Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    if (j.m_type == e_gearJoint)
			//    {
			//        continue;
			//    }

			//    b2Settings.b2Log("{\n");
			//    j.Dump();
			//    b2Settings.b2Log("}\n");
			//}

			//// Second pass on joints, only gear joints.
			//for (b2Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    if (j.m_type != e_gearJoint)
			//    {
			//        continue;
			//    }

			//    b2Settings.b2Log("{\n");
			//    j.Dump();
			//    b2Settings.b2Log("}\n");
			//}

			//b2Settings.b2Log("b2Free(joints);\n");
			//b2Settings.b2Log("b2Free(bodies);\n");
			//b2Settings.b2Log("joints = null;\n");
			//b2Settings.b2Log("bodies = null;\n");
		}

		

		private void Solve(b2TimeStep step){
			throw new NotImplementedException();
			//m_profile.solveInit = 0.0f;
			//m_profile.solveVelocity = 0.0f;
			//m_profile.solvePosition = 0.0f;

			//// Size the island for the worst case.
			//b2Island island(m_bodyCount,
			//                m_contactManager.m_contactCount,
			//                m_jointCount,
			//                &m_stackAllocator,
			//                m_contactManager.m_contactListener);

			//// Clear all the island flags.
			//for (b2Body* b = m_bodyList; b; b = b.m_next)
			//{
			//    b.m_flags &= ~b2Body::e_islandFlag;
			//}
			//for (b2Contact* c = m_contactManager.m_contactList; c; c = c.m_next)
			//{
			//    c.m_flags &= ~b2Contact::e_islandFlag;
			//}
			//for (b2Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    j.m_islandFlag = false;
			//}

			//// Build and simulate all awake islands.
			//int stackSize = m_bodyCount;
			//b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
			//for (b2Body* seed = m_bodyList; seed; seed = seed.m_next)
			//{
			//    if (seed.m_flags & b2Body::e_islandFlag)
			//    {
			//        continue;
			//    }

			//    if (seed.IsAwake() == false || seed.IsActive() == false)
			//    {
			//        continue;
			//    }

			//    // The seed can be dynamic or kinematic.
			//    if (seed.GetType() == b2_staticBody)
			//    {
			//        continue;
			//    }

			//    // Reset island and stack.
			//    island.Clear();
			//    int stackCount = 0;
			//    stack[stackCount++] = seed;
			//    seed.m_flags |= b2Body::e_islandFlag;

			//    // Perform a depth first search (DFS) on the constraint graph.
			//    while (stackCount > 0)
			//    {
			//        // Grab the next body off the stack and add it to the island.
			//        b2Body* b = stack[--stackCount];
			//        Utilities.Assert(b.IsActive() == true);
			//        island.Add(b);

			//        // Make sure the body is awake.
			//        b.SetAwake(true);

			//        // To keep islands as small as possible, we don't
			//        // propagate islands across static bodies.
			//        if (b.GetType() == b2_staticBody)
			//        {
			//            continue;
			//        }

			//        // Search all contacts connected to this body.
			//        for (b2ContactEdge* ce = b.m_contactList; ce; ce = ce.next)
			//        {
			//            b2Contact* contact = ce.contact;

			//            // Has this contact already been added to an island?
			//            if (contact.m_flags & b2Contact::e_islandFlag)
			//            {
			//                continue;
			//            }

			//            // Is this contact solid and touching?
			//            if (contact.IsEnabled() == false ||
			//                contact.IsTouching() == false)
			//            {
			//                continue;
			//            }

			//            // Skip sensors.
			//            bool sensorA = contact.m_fixtureA.m_isSensor;
			//            bool sensorB = contact.m_fixtureB.m_isSensor;
			//            if (sensorA || sensorB)
			//            {
			//                continue;
			//            }

			//            island.Add(contact);
			//            contact.m_flags |= b2Contact::e_islandFlag;

			//            b2Body* other = ce.other;

			//            // Was the other body already added to this island?
			//            if (other.m_flags & b2Body::e_islandFlag)
			//            {
			//                continue;
			//            }

			//            Utilities.Assert(stackCount < stackSize);
			//            stack[stackCount++] = other;
			//            other.m_flags |= b2Body::e_islandFlag;
			//        }

			//        // Search all joints connect to this body.
			//        for (b2JointEdge* je = b.m_jointList; je; je = je.next)
			//        {
			//            if (je.joint.m_islandFlag == true)
			//            {
			//                continue;
			//            }

			//            b2Body* other = je.other;

			//            // Don't simulate joints connected to inactive bodies.
			//            if (other.IsActive() == false)
			//            {
			//                continue;
			//            }

			//            island.Add(je.joint);
			//            je.joint.m_islandFlag = true;

			//            if (other.m_flags & b2Body::e_islandFlag)
			//            {
			//                continue;
			//            }

			//            Utilities.Assert(stackCount < stackSize);
			//            stack[stackCount++] = other;
			//            other.m_flags |= b2Body::e_islandFlag;
			//        }
			//    }

			//    b2Profile profile;
			//    island.Solve(&profile, step, m_gravity, m_allowSleep);
			//    m_profile.solveInit += profile.solveInit;
			//    m_profile.solveVelocity += profile.solveVelocity;
			//    m_profile.solvePosition += profile.solvePosition;

			//    // Post solve cleanup.
			//    for (int i = 0; i < island.m_bodyCount; ++i)
			//    {
			//        // Allow static bodies to participate in other islands.
			//        b2Body* b = island.m_bodies[i];
			//        if (b.GetType() == b2_staticBody)
			//        {
			//            b.m_flags &= ~b2Body::e_islandFlag;
			//        }
			//    }
			//}

			//m_stackAllocator.Free(stack);

			//{
			//    b2Timer timer;
			//    // Synchronize fixtures, check for out of range bodies.
			//    for (b2Body* b = m_bodyList; b; b = b.GetNext())
			//    {
			//        // If a body was not in an island then it did not move.
			//        if ((b.m_flags & b2Body::e_islandFlag) == 0)
			//        {
			//            continue;
			//        }

			//        if (b.GetType() == b2_staticBody)
			//        {
			//            continue;
			//        }

			//        // Update fixtures (for broad-phase).
			//        b.SynchronizeFixtures();
			//    }

			//    // Look for new contacts.
			//    m_contactManager.FindNewContacts();
			//    m_profile.broadphase = timer.GetMilliseconds();
			//}
		}
		private void SolveTOI(b2TimeStep step){
			throw new NotImplementedException();
			//b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener);

			//if (m_stepComplete)
			//{
			//    for (b2Body* b = m_bodyList; b; b = b.m_next)
			//    {
			//        b.m_flags &= ~b2Body::e_islandFlag;
			//        b.m_sweep.alpha0 = 0.0f;
			//    }

			//    for (b2Contact* c = m_contactManager.m_contactList; c; c = c.m_next)
			//    {
			//        // Invalidate TOI
			//        c.m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			//        c.m_toiCount = 0;
			//        c.m_toi = 1.0f;
			//    }
			//}

			//// Find TOI events and solve them.
			//for (;;)
			//{
			//    // Find the first TOI.
			//    b2Contact* minContact = null;
			//    float minAlpha = 1.0f;

			//    for (b2Contact* c = m_contactManager.m_contactList; c; c = c.m_next)
			//    {
			//        // Is this contact disabled?
			//        if (c.IsEnabled() == false)
			//        {
			//            continue;
			//        }

			//        // Prevent excessive sub-stepping.
			//        if (c.m_toiCount > b2_maxSubSteps)
			//        {
			//            continue;
			//        }

			//        float alpha = 1.0f;
			//        if (c.m_flags & b2Contact::e_toiFlag)
			//        {
			//            // This contact has a valid cached TOI.
			//            alpha = c.m_toi;
			//        }
			//        else
			//        {
			//            b2Fixture* fA = c.GetFixtureA();
			//            b2Fixture* fB = c.GetFixtureB();

			//            // Is there a sensor?
			//            if (fA.IsSensor() || fB.IsSensor())
			//            {
			//                continue;
			//            }

			//            b2Body* bA = fA.GetBody();
			//            b2Body* bB = fB.GetBody();

			//            b2BodyType typeA = bA.m_type;
			//            b2BodyType typeB = bB.m_type;
			//            Utilities.Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

			//            bool activeA = bA.IsAwake() && typeA != b2_staticBody;
			//            bool activeB = bB.IsAwake() && typeB != b2_staticBody;

			//            // Is at least one body active (awake and dynamic or kinematic)?
			//            if (activeA == false && activeB == false)
			//            {
			//                continue;
			//            }

			//            bool collideA = bA.IsBullet() || typeA != b2_dynamicBody;
			//            bool collideB = bB.IsBullet() || typeB != b2_dynamicBody;

			//            // Are these two non-bullet dynamic bodies?
			//            if (collideA == false && collideB == false)
			//            {
			//                continue;
			//            }

			//            // Compute the TOI for this contact.
			//            // Put the sweeps onto the same time interval.
			//            float alpha0 = bA.m_sweep.alpha0;

			//            if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
			//            {
			//                alpha0 = bB.m_sweep.alpha0;
			//                bA.m_sweep.Advance(alpha0);
			//            }
			//            else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
			//            {
			//                alpha0 = bA.m_sweep.alpha0;
			//                bB.m_sweep.Advance(alpha0);
			//            }

			//            Utilities.Assert(alpha0 < 1.0f);

			//            int indexA = c.GetChildIndexA();
			//            int indexB = c.GetChildIndexB();

			//            // Compute the time of impact in interval [0, minTOI]
			//            b2TOIInput input;
			//            input.proxyA.Set(fA.GetShape(), indexA);
			//            input.proxyB.Set(fB.GetShape(), indexB);
			//            input.sweepA = bA.m_sweep;
			//            input.sweepB = bB.m_sweep;
			//            input.tMax = 1.0f;

			//            b2TOIOutput output;
			//            b2TimeOfImpact(&output, &input);

			//            // Beta is the fraction of the remaining portion of the .
			//            float beta = output.t;
			//            if (output.state == b2TOIOutput::e_touching)
			//            {
			//                alpha = Math.Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
			//            }
			//            else
			//            {
			//                alpha = 1.0f;
			//            }

			//            c.m_toi = alpha;
			//            c.m_flags |= b2Contact::e_toiFlag;
			//        }

			//        if (alpha < minAlpha)
			//        {
			//            // This is the minimum TOI found so far.
			//            minContact = c;
			//            minAlpha = alpha;
			//        }
			//    }

			//    if (minContact == null || 1.0f - 10.0f * Single.Epsilon < minAlpha)
			//    {
			//        // No more TOI events. Done!
			//        m_stepComplete = true;
			//        break;
			//    }

			//    // Advance the bodies to the TOI.
			//    b2Fixture* fA = minContact.GetFixtureA();
			//    b2Fixture* fB = minContact.GetFixtureB();
			//    b2Body* bA = fA.GetBody();
			//    b2Body* bB = fB.GetBody();

			//    b2Sweep backup1 = bA.m_sweep;
			//    b2Sweep backup2 = bB.m_sweep;

			//    bA.Advance(minAlpha);
			//    bB.Advance(minAlpha);

			//    // The TOI contact likely has some new contact points.
			//    minContact.Update(m_contactManager.m_contactListener);
			//    minContact.m_flags &= ~b2Contact::e_toiFlag;
			//    ++minContact.m_toiCount;

			//    // Is the contact solid?
			//    if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
			//    {
			//        // Restore the sweeps.
			//        minContact.SetEnabled(false);
			//        bA.m_sweep = backup1;
			//        bB.m_sweep = backup2;
			//        bA.SynchronizeTransform();
			//        bB.SynchronizeTransform();
			//        continue;
			//    }

			//    bA.SetAwake(true);
			//    bB.SetAwake(true);

			//    // Build the island
			//    island.Clear();
			//    island.Add(bA);
			//    island.Add(bB);
			//    island.Add(minContact);

			//    bA.m_flags |= b2Body::e_islandFlag;
			//    bB.m_flags |= b2Body::e_islandFlag;
			//    minContact.m_flags |= b2Contact::e_islandFlag;

			//    // Get contacts on bodyA and bodyB.
			//    b2Body* bodies[2] = {bA, bB};
			//    for (int i = 0; i < 2; ++i)
			//    {
			//        b2Body* body = bodies[i];
			//        if (body.m_type == b2_dynamicBody)
			//        {
			//            for (b2ContactEdge* ce = body.m_contactList; ce; ce = ce.next)
			//            {
			//                if (island.m_bodyCount == island.m_bodyCapacity)
			//                {
			//                    break;
			//                }

			//                if (island.m_contactCount == island.m_contactCapacity)
			//                {
			//                    break;
			//                }

			//                b2Contact* contact = ce.contact;

			//                // Has this contact already been added to the island?
			//                if (contact.m_flags & b2Contact::e_islandFlag)
			//                {
			//                    continue;
			//                }

			//                // Only add static, kinematic, or bullet bodies.
			//                b2Body* other = ce.other;
			//                if (other.m_type == b2_dynamicBody &&
			//                    body.IsBullet() == false && other.IsBullet() == false)
			//                {
			//                    continue;
			//                }

			//                // Skip sensors.
			//                bool sensorA = contact.m_fixtureA.m_isSensor;
			//                bool sensorB = contact.m_fixtureB.m_isSensor;
			//                if (sensorA || sensorB)
			//                {
			//                    continue;
			//                }

			//                // Tentatively advance the body to the TOI.
			//                b2Sweep backup = other.m_sweep;
			//                if ((other.m_flags & b2Body::e_islandFlag) == 0)
			//                {
			//                    other.Advance(minAlpha);
			//                }

			//                // Update the contact points
			//                contact.Update(m_contactManager.m_contactListener);

			//                // Was the contact disabled by the user?
			//                if (contact.IsEnabled() == false)
			//                {
			//                    other.m_sweep = backup;
			//                    other.SynchronizeTransform();
			//                    continue;
			//                }

			//                // Are there contact points?
			//                if (contact.IsTouching() == false)
			//                {
			//                    other.m_sweep = backup;
			//                    other.SynchronizeTransform();
			//                    continue;
			//                }

			//                // Add the contact to the island
			//                contact.m_flags |= b2Contact::e_islandFlag;
			//                island.Add(contact);

			//                // Has the other body already been added to the island?
			//                if (other.m_flags & b2Body::e_islandFlag)
			//                {
			//                    continue;
			//                }
					
			//                // Add the other body to the island.
			//                other.m_flags |= b2Body::e_islandFlag;

			//                if (other.m_type != b2_staticBody)
			//                {
			//                    other.SetAwake(true);
			//                }

			//                island.Add(other);
			//            }
			//        }
			//    }

			//    b2TimeStep subStep;
			//    subStep.dt = (1.0f - minAlpha) * step.dt;
			//    subStep.inv_dt = 1.0f / subStep.dt;
			//    subStep.dtRatio = 1.0f;
			//    subStep.positionIterations = 20;
			//    subStep.velocityIterations = step.velocityIterations;
			//    subStep.warmStarting = false;
			//    island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

			//    // Reset island flags and synchronize broad-phase proxies.
			//    for (int i = 0; i < island.m_bodyCount; ++i)
			//    {
			//        b2Body* body = island.m_bodies[i];
			//        body.m_flags &= ~b2Body::e_islandFlag;

			//        if (body.m_type != b2_dynamicBody)
			//        {
			//            continue;
			//        }

			//        body.SynchronizeFixtures();

			//        // Invalidate all contact TOIs on this displaced body.
			//        for (b2ContactEdge* ce = body.m_contactList; ce; ce = ce.next)
			//        {
			//            ce.contact.m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			//        }
			//    }

			//    // Commit fixture proxy movements to the broad-phase so that new contacts are created.
			//    // Also, some contacts can be destroyed.
			//    m_contactManager.FindNewContacts();

			//    if (m_subStepping)
			//    {
			//        m_stepComplete = false;
			//        break;
			//    }
			//}
		}


		private void DrawJoint(b2Joint joint)
		{
			throw new NotImplementedException();
			//b2Body* bodyA = joint.GetBodyA();
			//b2Body* bodyB = joint.GetBodyB();
			//const b2Transform& xf1 = bodyA.GetTransform();
			//const b2Transform& xf2 = bodyB.GetTransform();
			//b2Vec2 x1 = xf1.p;
			//b2Vec2 x2 = xf2.p;
			//b2Vec2 p1 = joint.GetAnchorA();
			//b2Vec2 p2 = joint.GetAnchorB();

			//b2Color color(0.5f, 0.8f, 0.8f);

			//switch (joint.GetType())
			//{
			//case e_distanceJoint:
			//    m_debugDraw.DrawSegment(p1, p2, color);
			//    break;

			//case e_pulleyJoint:
			//    {
			//        b2PulleyJoint* pulley = (b2PulleyJoint*)joint;
			//        b2Vec2 s1 = pulley.GetGroundAnchorA();
			//        b2Vec2 s2 = pulley.GetGroundAnchorB();
			//        m_debugDraw.DrawSegment(s1, p1, color);
			//        m_debugDraw.DrawSegment(s2, p2, color);
			//        m_debugDraw.DrawSegment(s1, s2, color);
			//    }
			//    break;

			//case e_mouseJoint:
			//    // don't draw this
			//    break;

			//default:
			//    m_debugDraw.DrawSegment(x1, p1, color);
			//    m_debugDraw.DrawSegment(p1, p2, color);
			//    m_debugDraw.DrawSegment(x2, p2, color);
			//}
		}
		private void DrawShape(b2Fixture shape, b2Transform xf, Color color){
			throw new NotImplementedException();
			//switch (fixture.GetType())
			//{
			//case b2Shape::e_circle:
			//    {
			//        b2CircleShape* circle = (b2CircleShape*)fixture.GetShape();

			//        b2Vec2 center = Utilities.b2Mul(xf, circle.m_p);
			//        float radius = circle.m_radius;
			//        b2Vec2 axis = Utilities.b2Mul(xf.q, b2Vec2(1.0f, 0.0f));

			//        m_debugDraw.DrawSolidCircle(center, radius, axis, color);
			//    }
			//    break;

			//case b2Shape::e_edge:
			//    {
			//        b2EdgeShape* edge = (b2EdgeShape*)fixture.GetShape();
			//        b2Vec2 v1 = Utilities.b2Mul(xf, edge.m_vertex1);
			//        b2Vec2 v2 = Utilities.b2Mul(xf, edge.m_vertex2);
			//        m_debugDraw.DrawSegment(v1, v2, color);
			//    }
			//    break;

			//case b2Shape::e_chain:
			//    {
			//        b2ChainShape* chain = (b2ChainShape*)fixture.GetShape();
			//        int count = chain.m_count;
			//        const b2Vec2* vertices = chain.m_vertices;

			//        b2Vec2 v1 = Utilities.b2Mul(xf, vertices[0]);
			//        for (int i = 1; i < count; ++i)
			//        {
			//            b2Vec2 v2 = Utilities.b2Mul(xf, vertices[i]);
			//            m_debugDraw.DrawSegment(v1, v2, color);
			//            m_debugDraw.DrawCircle(v1, 0.05f, color);
			//            v1 = v2;
			//        }
			//    }
			//    break;

			//case b2Shape::e_polygon:
			//    {
			//        b2PolygonShape* poly = (b2PolygonShape*)fixture.GetShape();
			//        int vertexCount = poly.m_count;
			//        Utilities.Assert(vertexCount <= b2Settings.b2_maxPolygonVertices);
			//        b2Vec2 vertices[b2Settings.b2_maxPolygonVertices];

			//        for (int i = 0; i < vertexCount; ++i)
			//        {
			//            vertices[i] = Utilities.b2Mul(xf, poly.m_vertices[i]);
			//        }

			//        m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
			//    }
			//    break;
            
			//default:
			//    break;
			//}
		}

		internal WorldFlags m_flags;

		internal b2ContactManager m_contactManager;

		private List<b2Body> m_bodyList;//pointer
		private List<b2Joint> m_jointList;//pointer

		private int m_jointCount;

		private b2Vec2 m_gravity;
		private bool m_allowSleep;

		private b2DestructionListener m_destructionListener; //pointer
		private b2Draw m_debugDraw;//pointer

		// This is used to compute the time step ratio to
		// support a variable time step.
		private float m_inv_dt0;

		// These are for debugging the solver.
		private bool m_warmStarting;
		private bool m_continuousPhysics;
		private bool m_subStepping;

		private bool m_stepComplete;

		private b2Profile m_profile;
	}
}
