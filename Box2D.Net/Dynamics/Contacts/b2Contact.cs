using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// The class manages contact between two shapes. A contact exists for each overlapping
	/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	/// that has no contact points.
	public abstract class b2Contact
	{
		protected uint m_flags;

		// World pool and list pointers.
		protected b2Contact m_prev; //pointer
		protected b2Contact m_next;//pointer

		// Nodes for connecting bodies.
		protected b2ContactEdge m_nodeA;
		protected b2ContactEdge m_nodeB;

		protected b2Fixture m_fixtureA;//pointer
		protected b2Fixture m_fixtureB;//pointer

		protected int m_indexA;
		protected int m_indexB;

		protected b2Manifold m_manifold;

		protected int m_toiCount;
		protected float m_toi;

		protected float m_friction;
		protected float m_restitution;

		protected float m_tangentSpeed;

		/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
		/// For example, anything slides on ice.
		public static float b2MixFriction(float friction1, float friction2)
		{
			return (float)Math.Sqrt(friction1 * friction2);
		}

		/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
		/// For example, a superball bounces on anything.
		public static float b2MixRestitution(float restitution1, float restitution2)
		{
			return restitution1 > restitution2 ? restitution1 : restitution2;
		}

		public delegate b2Contact b2ContactCreateFcn(b2Fixture fixtureA, int indexA, //figureA & fixtureB were both pointers
													b2Fixture fixtureB, int indexB);
		public delegate void b2ContactDestroyFcn(b2Contact contact);

		/// Get the contact manifold. Do not modify the manifold unless you understand the
		/// internals of Box2D.
		public b2Manifold GetManifold(){
			return m_manifold; //TODO return reference?
		}

		/// Get the world manifold.
		public void GetWorldManifold(b2WorldManifold worldManifold){
			throw new NotImplementedException();
			//const b2Body* bodyA = m_fixtureA.GetBody();
			//const b2Body* bodyB = m_fixtureB.GetBody();
			//const b2Shape* shapeA = m_fixtureA.GetShape();
			//const b2Shape* shapeB = m_fixtureB.GetShape();

			//worldManifold.Initialize(&m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
		}

		/// Is this contact touching?
		public bool IsTouching() {
			throw new NotImplementedException();
			//return (m_flags & e_touchingFlag) == e_touchingFlag;
		}

		/// Enable/disable this contact. This can be used inside the pre-solve
		/// contact listener. The contact is only disabled for the current
		/// time step (or sub-step in continuous collisions).
		public void SetEnabled(bool flag){
			throw new NotImplementedException();
			//if (flag)
			//{
			//    m_flags |= e_enabledFlag;
			//}
			//else
			//{
			//    m_flags &= ~e_enabledFlag;
			//}
		}

		/// Has this contact been disabled?
		public bool IsEnabled(){
			throw new NotImplementedException();
			//return (m_flags & e_enabledFlag) == e_enabledFlag;
		}

		/// Get the next contact in the world's contact list.
		public b2Contact GetNext(){
			return m_next;
		}

		/// Get fixture A in this contact.
		public b2Fixture GetFixtureA(){
			return m_fixtureA;
		}

		/// Get the child primitive index for fixture A.
		public int GetChildIndexA(){
			return m_indexA;
		}

		/// Get fixture B in this contact.
		public b2Fixture GetFixtureB(){
			return m_fixtureB;
		}

		/// Get the child primitive index for fixture B.
		public int GetChildIndexB(){
			return m_indexB;
		}

		/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
		/// This value persists until set or reset.
		public void SetFriction(float friction){
			m_friction = friction;
		}

		/// Get the friction.
		public float GetFriction(){
			return m_friction;
		}

		/// Reset the friction mixture to the default value.
		public void ResetFriction(){
			m_friction = b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
		}

		/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
		/// The value persists until you set or reset.
		public void SetRestitution(float restitution){
			m_restitution = restitution;
		}

		/// Get the restitution.
		public float GetRestitution(){
			return m_restitution;
		}

		/// Reset the restitution to the default value.
		public void ResetRestitution(){
			m_restitution = b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
		}

		/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
		public void SetTangentSpeed(float speed){
			m_tangentSpeed = speed;
		}

		/// Get the desired tangent speed. In meters per second.
		public float GetTangentSpeed(){
			return m_tangentSpeed;
		}

		/// Evaluate this contact with your own manifold and transforms.
		public abstract void Evaluate(b2Manifold manifold, b2Transform xfA, b2Transform xfB); //manifold was pointer

		// Flags stored in m_flags
		[Flags]
		protected enum ContactFlags
		{
			// Used when crawling contact graph when forming islands.
			e_islandFlag		= 0x0001,

			// Set when the shapes are touching.
			e_touchingFlag		= 0x0002,

			// This contact can be disabled (by user)
			e_enabledFlag		= 0x0004,

			// This contact needs filtering because a fixture filter was changed.
			e_filterFlag		= 0x0008,

			// This bullet contact had a TOI event
			e_bulletHitFlag		= 0x0010,

			// This contact has a valid TOI in m_toi
			e_toiFlag			= 0x0020
		};

		/// Flag this contact for filtering. Filtering will occur the next time step.
		protected void FlagForFiltering(){
			throw new NotImplementedException();
			//m_flags |= e_filterFlag;
		}

		protected static void AddType(b2ContactCreateFcn createFcn, b2ContactDestroyFcn destroyFcn,
							ShapeType typeA, ShapeType typeB){
			throw new NotImplementedException();
			//b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
			//b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
	
			//s_registers[type1][type2].createFcn = createFcn;
			//s_registers[type1][type2].destroyFcn = destoryFcn;
			//s_registers[type1][type2].primary = true;

			//if (type1 != type2)
			//{
			//    s_registers[type2][type1].createFcn = createFcn;
			//    s_registers[type2][type1].destroyFcn = destoryFcn;
			//    s_registers[type2][type1].primary = false;
			//}
		}

		protected static void InitializeRegisters(){
			throw new NotImplementedException();
			//AddType(b2CircleContact::Create, b2CircleContact::Destroy, b2Shape::e_circle, b2Shape::e_circle);
			//AddType(b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, b2Shape::e_polygon, b2Shape::e_circle);
			//AddType(b2PolygonContact::Create, b2PolygonContact::Destroy, b2Shape::e_polygon, b2Shape::e_polygon);
			//AddType(b2EdgeAndCircleContact::Create, b2EdgeAndCircleContact::Destroy, b2Shape::e_edge, b2Shape::e_circle);
			//AddType(b2EdgeAndPolygonContact::Create, b2EdgeAndPolygonContact::Destroy, b2Shape::e_edge, b2Shape::e_polygon);
			//AddType(b2ChainAndCircleContact::Create, b2ChainAndCircleContact::Destroy, b2Shape::e_chain, b2Shape::e_circle);
			//AddType(b2ChainAndPolygonContact::Create, b2ChainAndPolygonContact::Destroy, b2Shape::e_chain, b2Shape::e_polygon);
		}

		protected static b2Contact Create(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB){
			throw new NotImplementedException();
			//if (s_initialized == false)
			//{
			//    InitializeRegisters();
			//    s_initialized = true;
			//}

			//b2Shape::Type type1 = fixtureA.GetType();
			//b2Shape::Type type2 = fixtureB.GetType();

			//b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
			//b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
	
			//b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
			//if (createFcn)
			//{
			//    if (s_registers[type1][type2].primary)
			//    {
			//        return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
			//    }
			//    else
			//    {
			//        return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
			//    }
			//}
			//else
			//{
			//    return null;
			//}
		}

		//protected static void Destroy(b2Contact* contact, b2Shape::Type typeA, b2Shape::Type typeB, b2BlockAllocator* allocator);
		protected static void Destroy(b2Contact contact){
			throw new NotImplementedException();
			//b2Assert(s_initialized == true);

			//b2Fixture* fixtureA = contact.m_fixtureA;
			//b2Fixture* fixtureB = contact.m_fixtureB;

			//if (contact.m_manifold.pointCount > 0 &&
			//    fixtureA.IsSensor() == false &&
			//    fixtureB.IsSensor() == false)
			//{
			//    fixtureA.GetBody().SetAwake(true);
			//    fixtureB.GetBody().SetAwake(true);
			//}

			//b2Shape::Type typeA = fixtureA.GetType();
			//b2Shape::Type typeB = fixtureB.GetType();

			//b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);
			//b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);

			//b2ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
			//destroyFcn(contact, allocator);
		}

		protected b2Contact(){
			m_fixtureA = null;
			m_fixtureB = null;
		}

		protected b2Contact(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB){
			throw new NotImplementedException();
			//m_flags = e_enabledFlag;

			//m_fixtureA = fA;
			//m_fixtureB = fB;

			//m_indexA = indexA;
			//m_indexB = indexB;

			//m_manifold.pointCount = 0;

			//m_prev = null;
			//m_next = null;

			//m_nodeA.contact = null;
			//m_nodeA.prev = null;
			//m_nodeA.next = null;
			//m_nodeA.other = null;

			//m_nodeB.contact = null;
			//m_nodeB.prev = null;
			//m_nodeB.next = null;
			//m_nodeB.other = null;

			//m_toiCount = 0;

			//m_friction = b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
			//m_restitution = b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);

			//m_tangentSpeed = 0.0f;
		}
		~b2Contact() {}

		// Update the contact manifold and touching status.
		// Note: do not assume the fixture AABBs are overlapping or are valid.
		protected void Update(b2ContactListener listener){
			throw new NotImplementedException();
			//b2Manifold oldManifold = m_manifold;

			//// Re-enable this contact.
			//m_flags |= e_enabledFlag;

			//bool touching = false;
			//bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

			//bool sensorA = m_fixtureA.IsSensor();
			//bool sensorB = m_fixtureB.IsSensor();
			//bool sensor = sensorA || sensorB;

			//b2Body* bodyA = m_fixtureA.GetBody();
			//b2Body* bodyB = m_fixtureB.GetBody();
			//const b2Transform& xfA = bodyA.GetTransform();
			//const b2Transform& xfB = bodyB.GetTransform();

			//// Is this contact a sensor?
			//if (sensor)
			//{
			//    const b2Shape* shapeA = m_fixtureA.GetShape();
			//    const b2Shape* shapeB = m_fixtureB.GetShape();
			//    touching = b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

			//    // Sensors don't generate manifolds.
			//    m_manifold.pointCount = 0;
			//}
			//else
			//{
			//    Evaluate(&m_manifold, xfA, xfB);
			//    touching = m_manifold.pointCount > 0;

			//    // Match old contact ids to new contact ids and copy the
			//    // stored impulses to warm start the solver.
			//    for (int i = 0; i < m_manifold.pointCount; ++i)
			//    {
			//        b2ManifoldPoint* mp2 = m_manifold.points + i;
			//        mp2.normalImpulse = 0.0f;
			//        mp2.tangentImpulse = 0.0f;
			//        b2ContactID id2 = mp2.id;

			//        for (int j = 0; j < oldManifold.pointCount; ++j)
			//        {
			//            b2ManifoldPoint* mp1 = oldManifold.points + j;

			//            if (mp1.id.key == id2.key)
			//            {
			//                mp2.normalImpulse = mp1.normalImpulse;
			//                mp2.tangentImpulse = mp1.tangentImpulse;
			//                break;
			//            }
			//        }
			//    }

			//    if (touching != wasTouching)
			//    {
			//        bodyA.SetAwake(true);
			//        bodyB.SetAwake(true);
			//    }
			//}

			//if (touching)
			//{
			//    m_flags |= e_touchingFlag;
			//}
			//else
			//{
			//    m_flags &= ~e_touchingFlag;
			//}

			//if (wasTouching == false && touching == true && listener)
			//{
			//    listener.BeginContact(this);
			//}

			//if (wasTouching == true && touching == false && listener)
			//{
			//    listener.EndContact(this);
			//}

			//if (sensor == false && touching && listener)
			//{
			//    listener.PreSolve(this, &oldManifold);
			//}
		}

		protected static b2ContactRegister[,] s_registers = new b2ContactRegister[(int)ShapeType.Count, (int)ShapeType.Count];
		protected static bool s_initialized = false;

		
	}
}
