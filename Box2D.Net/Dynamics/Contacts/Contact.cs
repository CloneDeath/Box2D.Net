using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Flags stored in m_flags
	[Flags]
	internal enum ContactFlags {
		// Used when crawling contact graph when forming islands.
		e_islandFlag = 0x0001,

		// Set when the shapes are touching.
		e_touchingFlag = 0x0002,

		// This contact can be disabled (by user)
		e_enabledFlag = 0x0004,

		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag = 0x0008,

		// This bullet contact had a TOI event
		e_bulletHitFlag = 0x0010,

		// This contact has a valid TOI in m_toi
		e_toiFlag = 0x0020
	};

	/// The class manages contact between two shapes. A contact exists for each overlapping
	/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	/// that has no contact points.
	public abstract class Contact
	{
		internal ContactFlags m_flags;

		// World pool and list pointers.
		//protected Contact m_prev; //pointer
		//protected Contact m_next;//pointer

		// Nodes for connecting bodies.
		internal ContactEdge m_nodeA;
		internal ContactEdge m_nodeB;

		internal Fixture m_fixtureA;//pointer
		internal Fixture m_fixtureB;//pointer

		protected int m_indexA;
		protected int m_indexB;

		protected Manifold m_manifold;

		internal int m_toiCount;
		internal float m_toi;

		internal float m_friction;
		internal float m_restitution;

		internal float m_tangentSpeed;

		/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
		/// For example, anything slides on ice.
		public static float MixFriction(float friction1, float friction2)
		{
			return (float)Math.Sqrt(friction1 * friction2);
		}

		/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
		/// For example, a superball bounces on anything.
		public static float MixRestitution(float restitution1, float restitution2)
		{
			return restitution1 > restitution2 ? restitution1 : restitution2;
		}

		public delegate Contact ContactCreateFcn(Fixture fixtureA, int indexA, //figureA & fixtureB were both pointers
													Fixture fixtureB, int indexB);
		public delegate void ContactDestroyFcn(Contact contact);

		/// Get the contact manifold. Do not modify the manifold unless you understand the
		/// internals of Box2D.
		public Manifold GetManifold(){
			return m_manifold; //TODO return reference?
		}

		/// Get the world manifold.
		public void GetWorldManifold(out WorldManifold worldManifold){
			throw new NotImplementedException();
			//const Body* bodyA = m_fixtureA.GetBody();
			//const Body* bodyB = m_fixtureB.GetBody();
			//const Shape* shapeA = m_fixtureA.GetShape();
			//const Shape* shapeB = m_fixtureB.GetShape();

			//worldManifold.Initialize(&m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
		}

		/// Is this contact touching?
		public bool IsTouching() {
			return (m_flags & ContactFlags.e_touchingFlag) == ContactFlags.e_touchingFlag;
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
			return (m_flags & ContactFlags.e_enabledFlag) == ContactFlags.e_enabledFlag;
		}

		/// Get the next contact in the world's contact list.
		//public Contact GetNext(){
		//    return m_next;
		//}

		/// Get fixture A in this contact.
		public Fixture FixtureA {
			get {
				return m_fixtureA;
			}
		}

		/// Get the child primitive index for fixture A.
		public int GetChildIndexA(){
			return m_indexA;
		}

		/// Get fixture B in this contact.
		public Fixture FixtureB {
			get {
				return m_fixtureB;
			}
		}

		/// Get the child primitive index for fixture B.
		public int GetChildIndexB(){
			return m_indexB;
		}

		/// Override the default friction mixture. You can call this in ContactListener::PreSolve.
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
			m_friction = MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
		}

		/// Override the default restitution mixture. You can call this in ContactListener::PreSolve.
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
			m_restitution = MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
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
		public abstract void Evaluate(out Manifold manifold, Transform xfA, Transform xfB); //manifold was pointer

		

		/// Flag this contact for filtering. Filtering will occur the next time step.
		internal void FlagForFiltering(){
			m_flags |= ContactFlags.e_filterFlag;
		}

		protected static void AddType(ContactCreateFcn createFcn, ContactDestroyFcn destroyFcn, ShapeType type1, ShapeType type2){
			Utilities.Assert(0 <= (int)type1 && type1 < ShapeType.Count);
			Utilities.Assert(0 <= (int)type2 && type2 < ShapeType.Count);

			s_registers[(int)type1, (int)type2].createFcn = createFcn;
			s_registers[(int)type1, (int)type2].destroyFcn = destroyFcn;
			s_registers[(int)type1, (int)type2].primary = true;

			if (type1 != type2)
			{
				s_registers[(int)type2, (int)type1].createFcn = createFcn;
				s_registers[(int)type2, (int)type1].destroyFcn = destroyFcn;
				s_registers[(int)type2, (int)type1].primary = false;
			}
		}

		protected static void InitializeRegisters(){
			AddType(CircleContact.Create, CircleContact.Destroy, ShapeType.Circle, ShapeType.Circle);
			AddType(PolygonAndCircleContact.Create, PolygonAndCircleContact.Destroy, ShapeType.Polygon, ShapeType.Circle);
			AddType(PolygonContact.Create, PolygonContact.Destroy, ShapeType.Polygon, ShapeType.Polygon);
			AddType(EdgeAndCircleContact.Create, EdgeAndCircleContact.Destroy, ShapeType.Edge, ShapeType.Circle);
			AddType(EdgeAndPolygonContact.Create, EdgeAndPolygonContact.Destroy, ShapeType.Edge, ShapeType.Polygon);
			AddType(ChainAndCircleContact.Create, ChainAndCircleContact.Destroy, ShapeType.Chain, ShapeType.Circle);
			AddType(ChainAndPolygonContact.Create, ChainAndPolygonContact.Destroy, ShapeType.Chain, ShapeType.Polygon);
		}

		internal static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB){
			if (s_initialized == false)
			{
			    InitializeRegisters();
			    s_initialized = true;
			}

			ShapeType type1 = fixtureA.GetShapeType();
			ShapeType type2 = fixtureB.GetShapeType();

			Utilities.Assert(0 <= type1 && type1 < ShapeType.Count);
			Utilities.Assert(0 <= type2 && type2 < ShapeType.Count);
	
			ContactCreateFcn createFcn = s_registers[(int)type1, (int)type2].createFcn;
			if (createFcn != null)
			{
			    if (s_registers[(int)type1, (int)type2].primary)
			    {
			        return createFcn(fixtureA, indexA, fixtureB, indexB);
			    }
			    else
			    {
			        return createFcn(fixtureB, indexB, fixtureA, indexA);
			    }
			}
			else
			{
			    return null;
			}
		}

		//protected static void Destroy(Contact contact, Shape::Type typeA, Shape::Type typeB, BlockAllocator* allocator);
		protected static void Destroy(Contact contact){
			throw new NotImplementedException();
			//Utilities.Assert(s_initialized == true);

			//Fixture fixtureA = contact.m_fixtureA;
			//Fixture fixtureB = contact.m_fixtureB;

			//if (contact.m_manifold.pointCount > 0 &&
			//    fixtureA.IsSensor() == false &&
			//    fixtureB.IsSensor() == false)
			//{
			//    fixtureA.GetBody().SetAwake(true);
			//    fixtureB.GetBody().SetAwake(true);
			//}

			//Shape::Type typeA = fixtureA.GetType();
			//Shape::Type typeB = fixtureB.GetType();

			//Utilities.Assert(0 <= typeA && typeB < ShapeType.typeCount);
			//Utilities.Assert(0 <= typeA && typeB < ShapeType.typeCount);

			//ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
			//destroyFcn(contact, allocator);
		}

		protected Contact(){
			m_fixtureA = null;
			m_fixtureB = null;
		}

		protected Contact(Fixture fA, int indexA, Fixture fB, int indexB) {
			m_flags = ContactFlags.e_enabledFlag;

			m_fixtureA = fA;
			m_fixtureB = fB;

			m_indexA = indexA;
			m_indexB = indexB;

			m_manifold = new Manifold();
			m_manifold.points.Clear();

			m_nodeA.contact = null;
			m_nodeA.other = null;

			m_nodeB.contact = null;
			m_nodeB.other = null;

			m_toiCount = 0;

			m_friction = MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
			m_restitution = MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);

			m_tangentSpeed = 0.0f;
		}
		~Contact() {}

		// Update the contact manifold and touching status.
		// Note: do not assume the fixture AABBs are overlapping or are valid.
		internal void Update(ContactListener listener){
			Manifold oldManifold = m_manifold;

			// Re-enable this contact.
			m_flags |= ContactFlags.e_enabledFlag;

			bool touching = false;
			bool wasTouching = (m_flags & ContactFlags.e_touchingFlag) == ContactFlags.e_touchingFlag;

			bool sensorA = m_fixtureA.IsSensor;
			bool sensorB = m_fixtureB.IsSensor;
			bool sensor = sensorA || sensorB;

			Body bodyA = m_fixtureA.GetBody();
			Body bodyB = m_fixtureB.GetBody();
			Transform xfA = bodyA.GetTransform();
			Transform xfB = bodyB.GetTransform();

			// Is this contact a sensor?
			if (sensor)
			{
			    Shape shapeA = m_fixtureA.GetShape();
			    Shape shapeB = m_fixtureB.GetShape();
			    touching = Collision.TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

			    // Sensors don't generate manifolds.
				m_manifold.points.Clear();
			}
			else
			{
			    Evaluate(out m_manifold, xfA, xfB);
			    touching = m_manifold.points.Count() > 0;

			    // Match old contact ids to new contact ids and copy the
			    // stored impulses to warm start the solver.
			    for (int i = 0; i < m_manifold.points.Count(); ++i)
			    {
					ManifoldPoint mp2 = m_manifold.points[i];
					mp2.normalImpulse = 0.0f;
					mp2.tangentImpulse = 0.0f;
					ContactID id2 = mp2.id;

					for (int j = 0; j < oldManifold.points.Count(); ++j) {
						ManifoldPoint mp1 = oldManifold.points[j];

						if (mp1.id.key == id2.key) {
							mp2.normalImpulse = mp1.normalImpulse;
							mp2.tangentImpulse = mp1.tangentImpulse;
							break;
						}
					}
			    }

			    if (touching != wasTouching)
			    {
			        bodyA.SetAwake(true);
			        bodyB.SetAwake(true);
			    }
			}

			if (touching)
			{
			    m_flags |= ContactFlags.e_touchingFlag;
			}
			else
			{
				m_flags &= ~ContactFlags.e_touchingFlag;
			}

			if (wasTouching == false && touching == true && listener != null)
			{
			    listener.BeginContact(this);
			}

			if (wasTouching == true && touching == false && listener != null)
			{
			    listener.EndContact(this);
			}

			if (sensor == false && touching && listener != null)
			{
			    listener.PreSolve(this, oldManifold);
			}
		}

		protected static ContactRegister[,] s_registers = new ContactRegister[(int)ShapeType.Count, (int)ShapeType.Count];
		protected static bool s_initialized = false;

		
	}
}
