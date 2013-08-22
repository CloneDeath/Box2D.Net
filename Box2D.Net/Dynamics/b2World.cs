using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// The world class manages all physics entities, dynamic simulation,
	/// and asynchronous queries. The world also contains efficient memory
	/// management facilities.
	public class b2World {
		/// Construct a world object.
		/// @param gravity the world gravity vector.
		public b2World(const b2Vec2& gravity);

		/// Destruct the world. All physics entities are destroyed and all heap memory is released.
		public ~b2World();

		/// Register a destruction listener. The listener is owned by you and must
		/// remain in scope.
		public void SetDestructionListener(b2DestructionListener* listener);

		/// Register a contact filter to provide specific control over collision.
		/// Otherwise the default filter is used (b2_defaultFilter). The listener is
		/// owned by you and must remain in scope. 
		public void SetContactFilter(b2ContactFilter* filter);

		/// Register a contact event listener. The listener is owned by you and must
		/// remain in scope.
		public void SetContactListener(b2ContactListener* listener);

		/// Register a routine for debug drawing. The debug draw functions are called
		/// inside with b2World::DrawDebugData method. The debug draw object is owned
		/// by you and must remain in scope.
		public void SetDebugDraw(b2Draw* debugDraw);

		/// Create a rigid body given a definition. No reference to the definition
		/// is retained.
		/// @warning This function is locked during callbacks.
		public b2Body* CreateBody(const b2BodyDef* def);

		/// Destroy a rigid body given a definition. No reference to the definition
		/// is retained. This function is locked during callbacks.
		/// @warning This automatically deletes all associated shapes and joints.
		/// @warning This function is locked during callbacks.
		public void DestroyBody(b2Body* body);

		/// Create a joint to constrain bodies together. No reference to the definition
		/// is retained. This may cause the connected bodies to cease colliding.
		/// @warning This function is locked during callbacks.
		public b2Joint* CreateJoint(const b2JointDef* def);

		/// Destroy a joint. This may cause the connected bodies to begin colliding.
		/// @warning This function is locked during callbacks.
		public void DestroyJoint(b2Joint* joint);

		/// Take a time step. This performs collision detection, integration,
		/// and constraint solution.
		/// @param timeStep the amount of time to simulate, this should not vary.
		/// @param velocityIterations for the velocity constraint solver.
		/// @param positionIterations for the position constraint solver.
		public void Step(	float timeStep,
					int velocityIterations,
					int positionIterations);

		/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
		/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
		/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
		/// a fixed sized time step under a variable frame-rate.
		/// When you perform sub-stepping you will disable auto clearing of forces and instead call
		/// ClearForces after all sub-steps are complete in one pass of your game loop.
		/// @see SetAutoClearForces
		public void ClearForces();

		/// Call this to draw shapes and other debug draw data.
		public void DrawDebugData();

		/// Query the world for all fixtures that potentially overlap the
		/// provided AABB.
		/// @param callback a user implemented callback class.
		/// @param aabb the query box.
		public void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;

		/// Ray-cast the world for all fixtures in the path of the ray. Your callback
		/// controls whether you get the closest point, any point, or n-points.
		/// The ray-cast ignores shapes that contain the starting point.
		/// @param callback a user implemented callback class.
		/// @param point1 the ray starting point
		/// @param point2 the ray ending point
		public void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const;

		/// Get the world body list. With the returned body, use b2Body::GetNext to get
		/// the next body in the world list. A null body indicates the end of the list.
		/// @return the head of the world body list.
		public b2Body* GetBodyList(){
			return m_bodyList;
		}
		public const b2Body* GetBodyList() const{
			return m_bodyList;
		}

		/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
		/// the next joint in the world list. A null joint indicates the end of the list.
		/// @return the head of the world joint list.
		public b2Joint* GetJointList(){
			return m_jointList;
		}
		public const b2Joint* GetJointList() const{
			return m_jointList;
		}

		/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
		/// the next contact in the world list. A null contact indicates the end of the list.
		/// @return the head of the world contact list.
		/// @warning contacts are created and destroyed in the middle of a time step.
		/// Use b2ContactListener to avoid missing contacts.
		public b2Contact* GetContactList(){
			return m_contactManager.m_contactList;
		}
		public const b2Contact* GetContactList() const{
			return m_contactManager.m_contactList;
		}

		/// Enable/disable sleep.
		public void SetAllowSleeping(bool flag);
		public bool GetAllowSleeping() const { return m_allowSleep; }

		/// Enable/disable warm starting. For testing.
		public void SetWarmStarting(bool flag) { m_warmStarting = flag; }
		public bool GetWarmStarting() const { return m_warmStarting; }

		/// Enable/disable continuous physics. For testing.
		public void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
		public bool GetContinuousPhysics() const { return m_continuousPhysics; }

		/// Enable/disable single stepped continuous physics. For testing.
		public void SetSubStepping(bool flag) { m_subStepping = flag; }
		public bool GetSubStepping() const { return m_subStepping; }

		/// Get the number of broad-phase proxies.
		public int GetProxyCount() const;

		/// Get the number of bodies.
		public int GetBodyCount() const{
			return m_bodyCount;
		}

		/// Get the number of joints.
		public int GetJointCount() const{
			return m_jointCount;
		}

		/// Get the number of contacts (each may have 0 or more contact points).
		public int GetContactCount() const{
			return m_contactManager.m_contactCount;
		}

		/// Get the height of the dynamic tree.
		public int GetTreeHeight() const;

		/// Get the balance of the dynamic tree.
		public int GetTreeBalance() const;

		/// Get the quality metric of the dynamic tree. The smaller the better.
		/// The minimum is 1.
		public float GetTreeQuality() const;

		/// Change the global gravity vector.
		public void SetGravity(const b2Vec2& gravity){
			m_gravity = gravity;
		}
	
		/// Get the global gravity vector.
		public b2Vec2 GetGravity() const{
			return m_gravity;
		}

		/// Is the world locked (in the middle of a time step).
		public bool IsLocked() const{
			return (m_flags & e_locked) == e_locked;
		}

		/// Set flag to control automatic clearing of forces after each time step.
		public void SetAutoClearForces(bool flag){
			if (flag)
			{
				m_flags |= e_clearForces;
			}
			else
			{
				m_flags &= ~e_clearForces;
			}
		}

		/// Get the flag that controls automatic clearing of forces after each time step.
		public bool GetAutoClearForces() const{
			return (m_flags & e_clearForces) == e_clearForces;
		}

		/// Shift the world origin. Useful for large worlds.
		/// The body shift formula is: position -= newOrigin
		/// @param newOrigin the new origin with respect to the old origin
		public void ShiftOrigin(const b2Vec2& newOrigin);

		/// Get the contact manager for testing.
		public const b2ContactManager& GetContactManager() const{
			return m_contactManager;
		}

		/// Get the current profile.
		public const b2Profile& GetProfile() const{
			return m_profile;
		}

		/// Dump the world into the log file.
		/// @warning this should be called outside of a time step.
		public void Dump();

		// m_flags
		private enum
		{
			e_newFixture	= 0x0001,
			e_locked		= 0x0002,
			e_clearForces	= 0x0004
		};

		friend class b2Body;
		friend class b2Fixture;
		friend class b2ContactManager;
		friend class b2Controller;

		private void Solve(const b2TimeStep& step);
		private void SolveTOI(const b2TimeStep& step);

		private void DrawJoint(b2Joint* joint);
		private void DrawShape(b2Fixture* shape, const b2Transform& xf, const b2Color& color);

		private b2BlockAllocator m_blockAllocator;
		private b2StackAllocator m_stackAllocator;

		private int m_flags;

		private b2ContactManager m_contactManager;

		private b2Body* m_bodyList;
		private b2Joint* m_jointList;

		private int m_bodyCount;
		private int m_jointCount;

		private b2Vec2 m_gravity;
		private bool m_allowSleep;

		private b2DestructionListener* m_destructionListener;
		private b2Draw* m_debugDraw;

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
