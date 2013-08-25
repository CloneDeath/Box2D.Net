using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Diagnostics;

namespace Box2D {
	// m_flags
	[Flags]
	internal enum WorldFlags {
		e_newFixture = 0x0001,
		e_locked = 0x0002,
		e_clearForces = 0x0004
	}

	/// The world class manages all physics entities, dynamic simulation,
	/// and asynchronous queries. The world also contains efficient memory
	/// management facilities.
	public class World {
		/// Construct a world object.
		/// @param gravity the world gravity vector.
		public World(Vec2 gravity){
			m_destructionListener = null;
			m_debugDraw = null;

			m_bodyList = new List<Body>();
			m_jointList = new List<Joint>();

			m_warmStarting = true;
			m_continuousPhysics = true;
			m_subStepping = false;

			m_stepComplete = true;

			m_allowSleep = true;
			m_gravity = gravity;

			m_flags = WorldFlags.e_clearForces;

			m_inv_dt0 = 0.0f;

			m_profile = new Profile();
			m_contactManager = new ContactManager();
		}

		/// Register a destruction listener. The listener is owned by you and must
		/// remain in scope.
		public void SetDestructionListener(DestructionListener listener){
			m_destructionListener = listener;
		}

		/// Register a contact filter to provide specific control over collision.
		/// Otherwise the default filter is used (_defaultFilter). The listener is
		/// owned by you and must remain in scope. 
		public void SetContactFilter(ContactFilter filter){
			m_contactManager.m_contactFilter = filter;
		}

		/// Register a contact event listener. The listener is owned by you and must
		/// remain in scope.
		public void SetContactListener(ContactListener listener){
			m_contactManager.m_contactListener = listener;
		}

		/// Register a routine for debug drawing. The debug draw functions are called
		/// inside with World::DrawDebugData method. The debug draw object is owned
		/// by you and must remain in scope.
		public void SetDebugDraw(Draw debugDraw){
			m_debugDraw = debugDraw;
		}

		/// Create a rigid body given a definition. No reference to the definition
		/// is retained.
		/// @warning This function is locked during callbacks.
		public Body CreateBody(BodyDef def){
			Utilities.Assert(IsLocked() == false);
			if (IsLocked())
			{
			    return null;
			}
			
			Body b = new Body(def, this);

			// Add to world doubly linked list.
			m_bodyList.Add(b);

			return b;
		}

		/// Destroy a rigid body given a definition. No reference to the definition
		/// is retained. This function is locked during callbacks.
		/// @warning This automatically deletes all associated shapes and joints.
		/// @warning This function is locked during callbacks.
		public void DestroyBody(Body body){
			throw new NotImplementedException();
			//Utilities.Assert(m_bodyList.Count() > 0);
			//Utilities.Assert(IsLocked() == false);
			//if (IsLocked())
			//{
			//    return;
			//}

			//// Delete the attached joints.
			//JointEdge* je = b.m_jointList;
			//while (je)
			//{
			//    JointEdge* je0 = je;
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
			//ContactEdge* ce = b.m_contactList;
			//while (ce)
			//{
			//    ContactEdge* ce0 = ce;
			//    ce = ce.next;
			//    m_contactManager.Destroy(ce0.contact);
			//}
			//b.m_contactList = null;

			//// Delete the attached fixtures. This destroys broad-phase proxies.
			//Fixture* f = b.m_fixtureList;
			//while (f)
			//{
			//    Fixture* f0 = f;
			//    f = f.m_next;

			//    if (m_destructionListener)
			//    {
			//        m_destructionListener.SayGoodbye(f0);
			//    }

			//    f0.DestroyProxies(&m_contactManager.m_broadPhase);
			//    f0.Destroy(&m_blockAllocator);
			//    f0.~Fixture();
			//    m_blockAllocator.Free(f0, sizeof(Fixture));

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

			//--m_bodyList.Count();
			//b.~Body();
			//m_blockAllocator.Free(b, sizeof(Body));
		}

		/// Create a joint to constrain bodies together. No reference to the definition
		/// is retained. This may cause the connected bodies to cease colliding.
		/// @warning This function is locked during callbacks.
		public Joint CreateJoint(JointDef def){
			Utilities.Assert(IsLocked() == false);
			if (IsLocked())
			{
			    return null;
			}

			Joint j = Joint.Create(def);

			// Connect to the world list.
			m_jointList.Add(j);

			// Connect to the bodies' doubly linked lists.
			j.m_edgeA.Clear();
			j.m_edgeA.Add(new JointEdge(j, j.m_bodyB));

			j.m_edgeA.AddRange(j.m_bodyA.m_jointList);
			j.m_bodyA.m_jointList = j.m_edgeA;

			j.m_edgeB.Clear();
			j.m_edgeB.Add(new JointEdge(j, j.m_bodyA));
			j.m_edgeB.AddRange(j.m_bodyB.m_jointList);
			j.m_bodyB.m_jointList = j.m_edgeB;

			Body bodyA = def.bodyA;
			Body bodyB = def.bodyB;

			// If the joint prevents collisions, then flag any contacts for filtering.
			if (def.collideConnected == false)
			{
				foreach (ContactEdge edge in bodyB.GetContactList())
			    {
			        if (edge.other == bodyA)
			        {
			            // Flag the contact for filtering at the next time step (where either
			            // body is awake).
			            edge.contact.FlagForFiltering();
			        }
			    }
			}

			// Note: creating a joint doesn't wake the bodies.

			return j;
		}

		/// Destroy a joint. This may cause the connected bodies to begin colliding.
		/// @warning This function is locked during callbacks.
		public void DestroyJoint(Joint joint){
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
			//Body* bodyA = j.m_bodyA;
			//Body* bodyB = j.m_bodyB;

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

			//Joint::Destroy(j, &m_blockAllocator);

			//Utilities.Assert(m_jointCount > 0);
			//--m_jointCount;

			//// If the joint prevents collisions, then flag any contacts for filtering.
			//if (collideConnected == false)
			//{
			//    ContactEdge* edge = bodyB.GetContactList();
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
			Timer stepTimer = new Timer();

			// If new fixtures were added, we need to find the new contacts.
			if (m_flags.HasFlag(WorldFlags.e_newFixture)) {
				m_contactManager.FindNewContacts();
				m_flags &= ~WorldFlags.e_newFixture;
			}

			m_flags |= WorldFlags.e_locked;

			TimeStep step;
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
				Timer timer = new Timer();
				m_contactManager.Collide();
				m_profile.collide = timer.GetMilliseconds();
			}

			// Integrate velocities, solve velocity constraints, and integrate positions.
			if (m_stepComplete && step.dt > 0.0f) {
				Timer timer = new Timer();
				Solve(step);
				m_profile.solve = timer.GetMilliseconds();
			}

			// Handle TOI events.
			if (m_continuousPhysics && step.dt > 0.0f) {
				Timer timer = new Timer();
				SolveTOI(step);
				m_profile.solveTOI = timer.GetMilliseconds();
			}

			if (step.dt > 0.0f) {
				m_inv_dt0 = step.inv_dt;
			}

			if (m_flags.HasFlag(WorldFlags.e_clearForces)) {
				ClearForces();
			}

			m_flags &= ~WorldFlags.e_locked;

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
			foreach (Body body in m_bodyList){
				body.m_force.SetZero();
				body.m_torque = 0.0f;
			}
		}

		/// Call this to draw shapes and other debug draw data.
		public void DrawDebugData(){
			if (m_debugDraw == null)
			{
			    return;
			}

			Draw.DrawFlags flags = m_debugDraw.GetFlags();

			if (flags.HasFlag(Draw.DrawFlags.e_shapeBit))
			{
				foreach(Body b in m_bodyList)
			    {
			        Transform xf = b.GetTransform();
					foreach (Fixture f in b.GetFixtureList())
			        {
			            if (b.IsActive() == false)
			            {
			                DrawShape(f, xf, Color.FromArgb(128, 128, 75));
			            }
			            else if (b.GetBodyType() == BodyType._staticBody)
			            {
							DrawShape(f, xf, Color.FromArgb(128, 225, 128));
			            }
			            else if (b.GetBodyType() == BodyType._kinematicBody)
			            {
							DrawShape(f, xf, Color.FromArgb(128, 128, 225));
			            }
			            else if (b.IsAwake() == false)
			            {
							DrawShape(f, xf, Color.FromArgb(150, 150, 150));
			            }
			            else
			            {
							DrawShape(f, xf, Color.FromArgb(225, 175, 175));
			            }
			        }
			    }
			}

			if (flags.HasFlag(Draw.DrawFlags.e_jointBit))
			{
				foreach (Joint j in m_jointList)
			    {
			        DrawJoint(j);
			    }
			}

			if (flags.HasFlag(Draw.DrawFlags.e_pairBit))
			{
			    Color color = Color.FromArgb(75, 225, 225);
				foreach (Contact c in m_contactManager.m_contactList)
			    {
			        //Fixture fixtureA = c.FixtureA;
			        //Fixture fixtureB = c.FixtureB;

			        //Vec2 cA = fixtureA.GetAABB().GetCenter();
			        //Vec2 cB = fixtureB.GetAABB().GetCenter();

			        //m_debugDraw.DrawSegment(cA, cB, color);
			    }
			}

			if (flags.HasFlag(Draw.DrawFlags.e_aabbBit))
			{
			    Color color = Color.FromArgb(225, 75, 225);
			    BroadPhase bp = m_contactManager.m_broadPhase;

				foreach (Body b in m_bodyList)
			    {
			        if (b.IsActive() == false)
			        {
			            continue;
			        }

					foreach (Fixture f in b.GetFixtureList())
			        {
			            for (int i = 0; i < f.m_proxies.Count(); ++i)
			            {
			                FixtureProxy proxy = f.m_proxies[i];
			                AABB aabb = bp.GetFatAABB(proxy.proxyId);
			                Vec2[] vs = new Vec2[4];
			                vs[0].Set(aabb.lowerBound.X, aabb.lowerBound.Y);
			                vs[1].Set(aabb.upperBound.X, aabb.lowerBound.Y);
			                vs[2].Set(aabb.upperBound.X, aabb.upperBound.Y);
			                vs[3].Set(aabb.lowerBound.X, aabb.upperBound.Y);

			                m_debugDraw.DrawPolygon(vs, 4, color);
			            }
			        }
			    }
			}

			if (flags.HasFlag(Draw.DrawFlags.e_centerOfMassBit))
			{
				foreach(Body b in m_bodyList)
			    {
			        Transform xf = b.GetTransform();
			        xf.p = b.GetWorldCenter();
			        m_debugDraw.DrawTransform(xf);
			    }
			}
		}

		/// Query the world for all fixtures that potentially overlap the
		/// provided AABB.
		/// @param callback a user implemented callback class.
		/// @param aabb the query box.
		public void QueryAABB(QueryCallback callback, AABB aabb){
			throw new NotImplementedException();
			//WorldQueryWrapper wrapper;
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
		public void RayCast(RayCastCallback callback, Vec2 point1, Vec2 point2){
			throw new NotImplementedException();
			//WorldRayCastWrapper wrapper;
			//wrapper.broadPhase = &m_contactManager.m_broadPhase;
			//wrapper.callback = callback;
			//RayCastInput input;
			//input.maxFraction = 1.0f;
			//input.p1 = point1;
			//input.p2 = point2;
			//m_contactManager.m_broadPhase.RayCast(&wrapper, input);
		}

		/// Get the world body list. With the returned body, use Body::GetNext to get
		/// the next body in the world list. A null body indicates the end of the list.
		/// @return the head of the world body list.
		public List<Body> GetBodyList(){
			return m_bodyList;
		}

		/// Get the world joint list. With the returned joint, use Joint::GetNext to get
		/// the next joint in the world list. A null joint indicates the end of the list.
		/// @return the head of the world joint list.
		public List<Joint> GetJointList(){
			throw new NotImplementedException();
			//return m_jointList;
		}

		/// Get the world contact list. With the returned contact, use Contact::GetNext to get
		/// the next contact in the world list. A null contact indicates the end of the list.
		/// @return the head of the world contact list.
		/// @warning contacts are created and destroyed in the middle of a time step.
		/// Use ContactListener to avoid missing contacts.
		public List<Contact> GetContactList(){
			throw new NotImplementedException();
			//return m_contactManager.m_contactList;
		}

		/// Enable/disable sleep.
		public void SetAllowSleeping(bool flag){
			if (flag == m_allowSleep) {
				return;
			}

			m_allowSleep = flag;
			if (m_allowSleep == false) {
				foreach (Body b in m_bodyList) {
					b.SetAwake(true);
				}
			}
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
			return m_jointList.Count();
		}

		/// Get the number of contacts (each may have 0 or more contact points).
		public int GetContactCount(){
			return m_contactManager.m_contactList.Count();
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
		public void SetGravity(Vec2 gravity){
			m_gravity = gravity;
		}
	
		/// Get the global gravity vector.
		public Vec2 GetGravity(){
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
		public void ShiftOrigin(Vec2 newOrigin){
			throw new NotImplementedException();
			//Utilities.Assert((m_flags & WorldFlags.e_locked) == 0);
			//if ((m_flags & e_locked) == e_locked)
			//{
			//    return;
			//}

			//foreach (Body b in m_bodyList)
			//{
			//    b.m_xf.p -= newOrigin;
			//    b.m_sweep.c0 -= newOrigin;
			//    b.m_sweep.c -= newOrigin;
			//}

			//for (Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    j.ShiftOrigin(newOrigin);
			//}

			//m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
		}

		/// Get the contact manager for testing.
		public ContactManager GetContactManager(){
			return m_contactManager;
		}

		/// Get the current profile.
		public Profile GetProfile(){
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

			//Settings.Log("Vec2 g(%.15lef, %.15lef);\n", m_gravity.X, m_gravity.Y);
			//Settings.Log("m_world.SetGravity(g);\n");

			//Settings.Log("Body** bodies = (Body**)Alloc(%d * sizeof(Body*));\n", m_bodyList.Count());
			//Settings.Log("Joint** joints = (Joint**)Alloc(%d * sizeof(Joint*));\n", m_jointCount);
			//int i = 0;
			//foreach (Body b in m_bodyList)
			//{
			//    b.m_islandIndex = i;
			//    b.Dump();
			//    ++i;
			//}

			//i = 0;
			//for (Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    j.m_index = i;
			//    ++i;
			//}

			//// First pass on joints, skip gear joints.
			//for (Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    if (j.m_type == JointType.e_gearJoint)
			//    {
			//        continue;
			//    }

			//    Settings.Log("{\n");
			//    j.Dump();
			//    Settings.Log("}\n");
			//}

			//// Second pass on joints, only gear joints.
			//for (Joint* j = m_jointList; j; j = j.m_next)
			//{
			//    if (j.m_type != JointType.e_gearJoint)
			//    {
			//        continue;
			//    }

			//    Settings.Log("{\n");
			//    j.Dump();
			//    Settings.Log("}\n");
			//}

			//Settings.Log("Free(joints);\n");
			//Settings.Log("Free(bodies);\n");
			//Settings.Log("joints = null;\n");
			//Settings.Log("bodies = null;\n");
		}

		

		private void Solve(TimeStep step){
			m_profile.solveInit = 0.0f;
			m_profile.solveVelocity = 0.0f;
			m_profile.solvePosition = 0.0f;

			// Size the island for the worst case.
			Island island = new Island(m_contactManager.m_contactListener);

			// Clear all the island flags.
			foreach (Body b in m_bodyList)
			{
			    b.m_flags &= ~Body.BodyFlags.e_islandFlag;
			}
			foreach (Contact c in m_contactManager.m_contactList)
			{
			    c.m_flags &= ~ContactFlags.e_islandFlag;
			}
			foreach (Joint j in m_jointList)
			{
			    j.m_islandFlag = false;
			}

			// Build and simulate all awake islands.
			List<Body> stack = new List<Body>(m_bodyList.Count());
			foreach (Body seed in m_bodyList)
			{
			    if (seed.m_flags.HasFlag(Body.BodyFlags.e_islandFlag))
			    {
			        continue;
			    }

			    if (seed.IsAwake() == false || seed.IsActive() == false)
			    {
			        continue;
			    }

			    // The seed can be dynamic or kinematic.
			    if (seed.GetBodyType() == BodyType._staticBody)
			    {
			        continue;
			    }

			    // Reset island and stack.
			    island.Clear();
			    int stackCount = 0;
				stack.Add(seed); stackCount++;
			    seed.m_flags |= Body.BodyFlags.e_islandFlag;

			    // Perform a depth first search (DFS) on the constraint graph.
			    while (stackCount > 0)
			    {
			        // Grab the next body off the stack and add it to the island.
			        Body b = stack[--stackCount];
			        Utilities.Assert(b.IsActive() == true);
			        island.Add(b);

			        // Make sure the body is awake.
			        b.SetAwake(true);

			        // To keep islands as small as possible, we don't
			        // propagate islands across static bodies.
			        if (b.GetBodyType() == BodyType._staticBody)
			        {
			            continue;
			        }

			        // Search all contacts connected to this body.
			        foreach (ContactEdge ce in b.m_contactList)
			        {
			            Contact contact = ce.contact;

			            // Has this contact already been added to an island?
			            if (contact.m_flags.HasFlag(ContactFlags.e_islandFlag))
			            {
			                continue;
			            }

			            // Is this contact solid and touching?
			            if (contact.IsEnabled() == false ||
			                contact.IsTouching() == false)
			            {
			                continue;
			            }

			            // Skip sensors.
			            bool sensorA = contact.m_fixtureA.m_isSensor;
			            bool sensorB = contact.m_fixtureB.m_isSensor;
			            if (sensorA || sensorB)
			            {
			                continue;
			            }

			            island.Add(contact);
			            contact.m_flags |= ContactFlags.e_islandFlag;

			            Body other = ce.other;

			            // Was the other body already added to this island?
			            if (other.m_flags.HasFlag(Body.BodyFlags.e_islandFlag))
			            {
			                continue;
			            }

			            Utilities.Assert(stackCount < m_bodyList.Count());
						stack.Add(other); stackCount++;
			            other.m_flags |= Body.BodyFlags.e_islandFlag;
			        }

					// Search all joints connect to this body.
					foreach (JointEdge je in b.m_jointList){
						if (je.joint.m_islandFlag == true) {
							continue;
						}

						Body other = je.other;

						// Don't simulate joints connected to inactive bodies.
						if (other.IsActive() == false) {
							continue;
						}

						island.Add(je.joint);
						je.joint.m_islandFlag = true;

						if (other.m_flags.HasFlag(Body.BodyFlags.e_islandFlag)) {
							continue;
						}

						stack.Add(other); stackCount++;
						other.m_flags |= Body.BodyFlags.e_islandFlag;
					}
			    }

			    Profile profile = new Profile();
				island.Solve(profile, step, m_gravity, m_allowSleep);
				m_profile.solveInit += profile.solveInit;
				m_profile.solveVelocity += profile.solveVelocity;
				m_profile.solvePosition += profile.solvePosition;

				// Post solve cleanup.
				for (int i = 0; i < island.m_bodies.Count(); ++i) {
					// Allow static bodies to participate in other islands.
					Body b = island.m_bodies[i];
					if (b.GetBodyType() == BodyType._staticBody) {
						b.m_flags &= ~Body.BodyFlags.e_islandFlag;
					}
				}
			}

			{
			    Timer timer = new Timer();
			    // Synchronize fixtures, check for out of range bodies.
			    foreach (Body b in m_bodyList)
			    {
			        // If a body was not in an island then it did not move.
			        if ((b.m_flags & Body.BodyFlags.e_islandFlag) == 0)
			        {
			            continue;
			        }

			        if (b.GetBodyType() == BodyType._staticBody)
			        {
			            continue;
			        }

			        // Update fixtures (for broad-phase).
			        b.SynchronizeFixtures();
			    }

			    // Look for new contacts.
			    m_contactManager.FindNewContacts();
			    m_profile.broadphase = timer.GetMilliseconds();
			}
		}
		private void SolveTOI(TimeStep step){
			Island island = new Island(m_contactManager.m_contactListener);

			if (m_stepComplete)
			{
			    foreach (Body b in m_bodyList)
			    {
			        b.m_flags &= ~Body.BodyFlags.e_islandFlag;
			        b.m_sweep.alpha0 = 0.0f;
			    }

			    foreach (Contact c in m_contactManager.m_contactList)
			    {
			        // Invalidate TOI
			        c.m_flags &= ~(ContactFlags.e_toiFlag | ContactFlags.e_islandFlag);
			        c.m_toiCount = 0;
			        c.m_toi = 1.0f;
			    }
			}

			Fixture fA = null;
			Fixture fB = null;
			Body bA = null;
			Body bB = null;

			// Find TOI events and solve them.
			for (;;)
			{
			    // Find the first TOI.
			    Contact minContact = null;
			    float minAlpha = 1.0f;

			    foreach (Contact c in m_contactManager.m_contactList)
			    {
			        // Is this contact disabled?
			        if (c.IsEnabled() == false)
			        {
			            continue;
			        }

			        // Prevent excessive sub-stepping.
			        if (c.m_toiCount > Settings._maxSubSteps)
			        {
			            continue;
			        }

					

			        float alpha = 1.0f;
			        if (c.m_flags.HasFlag(ContactFlags.e_toiFlag))
			        {
			            // This contact has a valid cached TOI.
			            alpha = c.m_toi;
			        }
			        else
			        {
			            fA = c.FixtureA;
			            fB = c.FixtureB;

			            // Is there a sensor?
			            if (fA.IsSensor || fB.IsSensor)
			            {
			                continue;
			            }

			            bA = fA.GetBody();
			            bB = fB.GetBody();

			            BodyType typeA = bA.m_type;
			            BodyType typeB = bB.m_type;
			            Utilities.Assert(typeA == BodyType._dynamicBody || typeB == BodyType._dynamicBody);

			            bool activeA = bA.IsAwake() && typeA != BodyType._staticBody;
			            bool activeB = bB.IsAwake() && typeB != BodyType._staticBody;

			            // Is at least one body active (awake and dynamic or kinematic)?
			            if (activeA == false && activeB == false)
			            {
			                continue;
			            }

			            bool collideA = bA.IsBullet() || typeA != BodyType._dynamicBody;
			            bool collideB = bB.IsBullet() || typeB != BodyType._dynamicBody;

			            // Are these two non-bullet dynamic bodies?
			            if (collideA == false && collideB == false)
			            {
			                continue;
			            }

			            // Compute the TOI for this contact.
			            // Put the sweeps onto the same time interval.
			            float alpha0 = bA.m_sweep.alpha0;

			            if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
			            {
			                alpha0 = bB.m_sweep.alpha0;
			                bA.m_sweep.Advance(alpha0);
			            }
			            else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
			            {
			                alpha0 = bA.m_sweep.alpha0;
			                bB.m_sweep.Advance(alpha0);
			            }

			            Utilities.Assert(alpha0 < 1.0f);

			            int indexA = c.GetChildIndexA();
			            int indexB = c.GetChildIndexB();

			            // Compute the time of impact in interval [0, minTOI]
			            TOIInput input = new TOIInput();
			            input.proxyA.Set(fA.GetShape(), indexA);
			            input.proxyB.Set(fB.GetShape(), indexB);
			            input.sweepA = bA.m_sweep;
			            input.sweepB = bB.m_sweep;
			            input.tMax = 1.0f;

			            TOIOutput output;
			            Utilities.TimeOfImpact(out output, input);

			            // Beta is the fraction of the remaining portion of the .
			            float beta = output.t;
			            if (output.state == TOIOutput.State.e_touching)
			            {
			                alpha = Math.Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
			            }
			            else
			            {
			                alpha = 1.0f;
			            }

			            c.m_toi = alpha;
			            c.m_flags |= ContactFlags.e_toiFlag;
			        }

			        if (alpha < minAlpha)
			        {
			            // This is the minimum TOI found so far.
			            minContact = c;
			            minAlpha = alpha;
			        }
			    }

			    if (minContact == null || 1.0f - 10.0f * Single.Epsilon < minAlpha)
			    {
			        // No more TOI events. Done!
			        m_stepComplete = true;
			        break;
			    }

			    // Advance the bodies to the TOI.
			    fA = minContact.FixtureA;
			    fB = minContact.FixtureB;
			    bA = fA.GetBody();
			    bB = fB.GetBody();

			    Sweep backup1 = bA.m_sweep;
			    Sweep backup2 = bB.m_sweep;

			    bA.Advance(minAlpha);
			    bB.Advance(minAlpha);

			    // The TOI contact likely has some new contact points.
			    minContact.Update(m_contactManager.m_contactListener);
			    minContact.m_flags &= ~ContactFlags.e_toiFlag;
			    ++minContact.m_toiCount;

			    // Is the contact solid?
			    if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
			    {
			        // Restore the sweeps.
			        minContact.SetEnabled(false);
			        bA.m_sweep = backup1;
			        bB.m_sweep = backup2;
			        bA.SynchronizeTransform();
			        bB.SynchronizeTransform();
			        continue;
			    }

			    bA.SetAwake(true);
			    bB.SetAwake(true);

			    // Build the island
			    island.Clear();
			    island.Add(bA);
			    island.Add(bB);
			    island.Add(minContact);

			    bA.m_flags |= Body.BodyFlags.e_islandFlag;
			    bB.m_flags |= Body.BodyFlags.e_islandFlag;
			    minContact.m_flags |= ContactFlags.e_islandFlag;

			    // Get contacts on bodyA and bodyB.
			    Body[] bodies = {bA, bB};
			    for (int i = 0; i < 2; ++i)
			    {
			        Body body = bodies[i];
			        if (body.m_type == BodyType._dynamicBody)
			        {
						foreach (ContactEdge ce in body.m_contactList)
			            {
							throw new NotImplementedException();

							//if (island.m_bodies.Count() == island.m_bodyCapacity)
							//{
							//    break;
							//}

							//if (island.m_bodies.Count() == island.m_contactCapacity)
							//{
							//    break;
							//}

							//Contact* contact = ce.contact;

							//// Has this contact already been added to the island?
							//if (contact.m_flags & ContactFlags.e_islandFlag)
							//{
							//    continue;
							//}

							//// Only add static, kinematic, or bullet bodies.
							//Body* other = ce.other;
							//if (other.m_type == _dynamicBody &&
							//    body.IsBullet() == false && other.IsBullet() == false)
							//{
							//    continue;
							//}

							//// Skip sensors.
							//bool sensorA = contact.m_fixtureA.m_isSensor;
							//bool sensorB = contact.m_fixtureB.m_isSensor;
							//if (sensorA || sensorB)
							//{
							//    continue;
							//}

							//// Tentatively advance the body to the TOI.
							//Sweep backup = other.m_sweep;
							//if ((other.m_flags & Body.BodyFlags.e_islandFlag) == 0)
							//{
							//    other.Advance(minAlpha);
							//}

							//// Update the contact points
							//contact.Update(m_contactManager.m_contactListener);

							//// Was the contact disabled by the user?
							//if (contact.IsEnabled() == false)
							//{
							//    other.m_sweep = backup;
							//    other.SynchronizeTransform();
							//    continue;
							//}

							//// Are there contact points?
							//if (contact.IsTouching() == false)
							//{
							//    other.m_sweep = backup;
							//    other.SynchronizeTransform();
							//    continue;
							//}

							//// Add the contact to the island
							//contact.m_flags |= ContactFlags.e_islandFlag;
							//island.Add(contact);

							//// Has the other body already been added to the island?
							//if (other.m_flags & Body.BodyFlags.e_islandFlag)
							//{
							//    continue;
							//}
					
							//// Add the other body to the island.
							//other.m_flags |= Body.BodyFlags.e_islandFlag;

							//if (other.m_type != _staticBody)
							//{
							//    other.SetAwake(true);
							//}

							//island.Add(other);
			            }
			        }
			    }

			    TimeStep subStep;
			    subStep.dt = (1.0f - minAlpha) * step.dt;
			    subStep.inv_dt = 1.0f / subStep.dt;
			    subStep.dtRatio = 1.0f;
			    subStep.positionIterations = 20;
			    subStep.velocityIterations = step.velocityIterations;
			    subStep.warmStarting = false;
			    island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

			    // Reset island flags and synchronize broad-phase proxies.
			    for (int i = 0; i < island.m_bodies.Count(); ++i)
			    {
					throw new NotImplementedException();
					//Body* body = island.m_bodies[i];
					//body.m_flags &= ~Body.BodyFlags.e_islandFlag;

					//if (body.m_type != _dynamicBody)
					//{
					//    continue;
					//}

					//body.SynchronizeFixtures();

					//// Invalidate all contact TOIs on this displaced body.
					//for (ContactEdge* ce = body.m_contactList; ce; ce = ce.next)
					//{
					//    ce.contact.m_flags &= ~(ContactFlags.e_toiFlag | ContactFlags.e_islandFlag);
					//}
			    }

			    // Commit fixture proxy movements to the broad-phase so that new contacts are created.
			    // Also, some contacts can be destroyed.
			    m_contactManager.FindNewContacts();

			    if (m_subStepping)
			    {
			        m_stepComplete = false;
			        break;
			    }
			}
		}


		private void DrawJoint(Joint joint)
		{
			Body bodyA = joint.GetBodyA();
			Body bodyB = joint.GetBodyB();
			Transform xf1 = bodyA.GetTransform();
			Transform xf2 = bodyB.GetTransform();
			Vec2 x1 = xf1.p;
			Vec2 x2 = xf2.p;
			Vec2 p1 = joint.GetAnchorA();
			Vec2 p2 = joint.GetAnchorB();

			Color color = Color.FromArgb(128, 200, 200);

			switch (joint.GetJointType())
			{
			case JointType.e_distanceJoint:
			    m_debugDraw.DrawSegment(p1, p2, color);
			    break;

			case JointType.e_pulleyJoint:
			    {
					throw new NotImplementedException();
					//PulleyJoint pulley = (PulleyJoint)joint;
					//Vec2 s1 = pulley.GetGroundAnchorA();
					//Vec2 s2 = pulley.GetGroundAnchorB();
					//m_debugDraw.DrawSegment(s1, p1, color);
					//m_debugDraw.DrawSegment(s2, p2, color);
					//m_debugDraw.DrawSegment(s1, s2, color);
			    }
			    break;

			case JointType.e_mouseJoint:
			    // don't draw this
			    break;

			default:
			    m_debugDraw.DrawSegment(x1, p1, color);
			    m_debugDraw.DrawSegment(p1, p2, color);
			    m_debugDraw.DrawSegment(x2, p2, color);
				break;
			}
		}
		private void DrawShape(Fixture fixture, Transform xf, Color color){
			switch (fixture.GetShapeType())
			{
			case ShapeType.Circle:
			    {
			        CircleShape circle = (CircleShape)fixture.GetShape();

			        Vec2 center = Utilities.Mul(xf, circle.m_p);
			        float radius = circle.m_radius;
			        Vec2 axis = Utilities.Mul(xf.q, new Vec2(1.0f, 0.0f));

			        m_debugDraw.DrawSolidCircle(center, radius, axis, color);
			    }
			    break;

			case ShapeType.Edge:
			    {
			        EdgeShape edge = (EdgeShape)fixture.GetShape();
			        Vec2 v1 = Utilities.Mul(xf, edge.m_vertex1);
			        Vec2 v2 = Utilities.Mul(xf, edge.m_vertex2);
			        m_debugDraw.DrawSegment(v1, v2, color);
			    }
			    break;

			case ShapeType.Chain:
			    {
			        ChainShape chain = (ChainShape)fixture.GetShape();
			        int count = chain.m_count;
			        List<Vec2> vertices = chain.m_vertices;

			        Vec2 v1 = Utilities.Mul(xf, vertices[0]);
			        for (int i = 1; i < count; ++i)
			        {
			            Vec2 v2 = Utilities.Mul(xf, vertices[i]);
			            m_debugDraw.DrawSegment(v1, v2, color);
			            m_debugDraw.DrawCircle(v1, 0.05f, color);
			            v1 = v2;
			        }
			    }
			    break;

			case ShapeType.Polygon:
			    {
			        PolygonShape poly = (PolygonShape)fixture.GetShape();
			        int vertexCount = poly.m_count;
			        Utilities.Assert(vertexCount <= Settings._maxPolygonVertices);
			        Vec2[] vertices = new Vec2[Settings._maxPolygonVertices];

			        for (int i = 0; i < vertexCount; ++i)
			        {
			            vertices[i] = Utilities.Mul(xf, poly.m_vertices[i]);
			        }

			        m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
			    }
			    break;
            
			default:
			    break;
			}
		}

		internal WorldFlags m_flags;

		internal ContactManager m_contactManager;

		private List<Body> m_bodyList;//pointer
		private List<Joint> m_jointList;//pointer

		private Vec2 m_gravity;
		private bool m_allowSleep;

		private DestructionListener m_destructionListener; //pointer
		private Draw m_debugDraw;//pointer

		// This is used to compute the time step ratio to
		// support a variable time step.
		private float m_inv_dt0;

		// These are for debugging the solver.
		private bool m_warmStarting;
		private bool m_continuousPhysics;
		private bool m_subStepping;

		private bool m_stepComplete;

		private Profile m_profile;
	}
}
