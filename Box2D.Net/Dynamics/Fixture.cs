using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A fixture is used to attach a shape to a body for collision detection. A fixture
	/// inherits its transform from its parent. Fixtures hold additional non-geometric data
	/// such as friction, collision filters, etc.
	/// Fixtures are created via Body::CreateFixture.
	/// @warning you cannot reuse fixtures.
	public class Fixture
	{
		internal float m_Density;
		protected Fixture m_next; //pointer
		internal Body m_body; //pointer
		protected Shape m_shape; //pointer
		internal float m_friction;
		internal float m_restitution;

		internal List<FixtureProxy> m_proxies; //pointer

		protected Filter m_filter;

		internal bool m_isSensor;

		protected object m_userData;

		/// Get the type of the child shape. You can use this to down cast to the concrete shape.
		/// @return the shape type.
		public ShapeType GetShapeType(){
			return m_shape.GetShapeType();
		}

		/// Get the child shape. You can modify the child shape, however you should not change the
		/// number of vertices because this will crash some collision caching mechanisms.
		/// Manipulating the shape may lead to non-physical behavior.
		public Shape GetShape(){
			return m_shape;
		}

		/// Is this fixture a sensor (non-solid)?
		/// true if the shape is a sensor, false if it is not
		public bool IsSensor {
			get {
				return m_isSensor;
			}
			set {
				if (value != m_isSensor) {
					m_body.SetAwake(true);
					m_isSensor = value;
				}
			}
		}

		/// Set the contact filtering data. This will not update contacts until the next time
		/// step when either parent body is active and awake.
		/// This automatically calls Refilter.
		public Filter Filter {
			get {
				return m_filter;
			}
			set {
				m_filter = value;
				Refilter();
			}
		}

		public Body Body {
			get {
				return m_body;
			}
		}

		/// Call this if you want to establish collision that was previously disabled by ContactFilter::ShouldCollide.
		public void Refilter(){
			throw new NotImplementedException();
			//if (m_body == null)
			//{
			//    return;
			//}

			//// Flag associated contacts for filtering.
			//ContactEdge* edge = m_body.GetContactList();
			//while (edge)
			//{
			//    Contact* contact = edge.contact;
			//    Fixture fixtureA = contact.FixtureA;
			//    Fixture fixtureB = contact.FixtureB;
			//    if (fixtureA == this || fixtureB == this)
			//    {
			//        contact.FlagForFiltering();
			//    }

			//    edge = edge.next;
			//}

			//World* world = m_body.GetWorld();

			//if (world == null)
			//{
			//    return;
			//}

			//// Touch each proxy so that new pairs may be created
			//BroadPhase* broadPhase = &world.m_contactManager.m_broadPhase;
			//for (int i = 0; i < m_proxyCount; ++i)
			//{
			//    broadPhase.TouchProxy(m_proxies[i].proxyId);
			//}
		}

		/// Get the parent body of this fixture. This is null if the fixture is not attached.
		/// @return the parent body.
		public Body GetBody(){
			return m_body;
		}

		/// Get the next fixture in the parent body's fixture list.
		/// @return the next shape.
		public Fixture GetNext(){
			return m_next;
		}

		/// Get/Set the user data that was assigned in the fixture definition. Use this to
		/// store your application specific data.
		public object UserData {
			get {
				return m_userData;
			}
			set {
				m_userData = value;
			}
		}

		/// Test a point for containment in this fixture.
		/// @param p a point in world coordinates.
		public bool TestPoint(Vec2 p){
			return m_shape.TestPoint(m_body.GetTransform(), p);
		}

		/// Cast a ray against this shape.
		/// @param output the ray-cast results.
		/// @param input the ray-cast input parameters.
		public bool RayCast(out RayCastOutput output, RayCastInput input, int childIndex){
			return m_shape.RayCast(out output, input, m_body.GetTransform(), childIndex);
		}

		/// Get the mass data for this fixture. The mass data is based on the Density and
		/// the shape. The rotational inertia is about the shape's origin. This operation
		/// may be expensive.
		public void GetMassData(out MassData massData){
			m_shape.ComputeMass(out massData, m_Density);
		}

		/// Set the Density of this fixture. This will _not_ automatically adjust the mass
		/// of the body. You must call Body::ResetMassData to update the body's mass.
		public void SetDensity(float Density){
			throw new NotImplementedException();
			//Utilities.Assert(Utilities.IsValid(Density) && Density >= 0.0f);
			//m_Density = Density;
		}

		/// Get the Density of this fixture.
		public float GetDensity(){
			return m_Density;
		}

		/// Get the coefficient of friction.
		public float GetFriction(){
			return m_friction;
		}


		/// Set the coefficient of friction. This will _not_ change the friction of
		/// existing contacts.
		public void SetFriction(float friction){
			m_friction = friction;
		}


		/// Get the coefficient of restitution.
		public float GetRestitution() {
			return m_restitution;
		}

		/// Set the coefficient of restitution. This will _not_ change the restitution of
		/// existing contacts.
		public void SetRestitution(float restitution){
			m_restitution = restitution;
		}

		/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
		/// If you need a more accurate AABB, compute it using the shape and
		/// the body transform.
		public AABB GetAABB(int childIndex){
			throw new NotImplementedException();
			//Utilities.Assert(0 <= childIndex && childIndex < m_proxyCount);
			//return m_proxies[childIndex].aabb;
		}

		/// Dump this fixture to the log file.
		public void Dump(int bodyIndex){
			throw new NotImplementedException();
			//Settings.Log("    FixtureDef fd = new FixtureDef();\n");
			//Settings.Log("    fd.friction = %.15lef;\n", m_friction);
			//Settings.Log("    fd.restitution = %.15lef;\n", m_restitution);
			//Settings.Log("    fd.Density = %.15lef;\n", m_Density);
			//Settings.Log("    fd.IsSensor = (bool)(%d);\n", m_isSensor);
			//Settings.Log("    fd.Filter.CategoryBits = (ushort)(%d);\n", m_filter.CategoryBits);
			//Settings.Log("    fd.Filter.MaskBits = (ushort)(%d);\n", m_filter.MaskBits);
			//Settings.Log("    fd.Filter.GroupIndex = (short)(%d);\n", m_filter.GroupIndex);

			//switch (m_shape.m_type)
			//{
			//case ShapeType.circle:
			//    {
			//        CircleShape* s = (CircleShape*)m_shape;
			//        Settings.Log("    CircleShape shape;\n");
			//        Settings.Log("    shape.m_radius = %.15lef;\n", s.m_radius);
			//        Settings.Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s.m_p.X, s.m_p.Y);
			//    }
			//    break;

			//case ShapeType.edge:
			//    {
			//        EdgeShape* s = (EdgeShape*)m_shape;
			//        Settings.Log("    EdgeShape shape;\n");
			//        Settings.Log("    shape.m_radius = %.15lef;\n", s.m_radius);
			//        Settings.Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s.m_vertex0.X, s.m_vertex0.Y);
			//        Settings.Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s.m_vertex1.X, s.m_vertex1.Y);
			//        Settings.Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s.m_vertex2.X, s.m_vertex2.Y);
			//        Settings.Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s.m_vertex3.X, s.m_vertex3.Y);
			//        Settings.Log("    shape.m_hasVertex0 = (bool)(%d);\n", s.m_hasVertex0);
			//        Settings.Log("    shape.m_hasVertex3 = (bool)(%d);\n", s.m_hasVertex3);
			//    }
			//    break;

			//case ShapeType.polygon:
			//    {
			//        PolygonShape* s = (PolygonShape*)m_shape;
			//        Settings.Log("    PolygonShape shape;\n");
			//        Settings.Log("    Vec2[] vs = new Vec2[%d];\n", Settings._maxPolygonVertices);
			//        for (int i = 0; i < s.m_count; ++i)
			//        {
			//            Settings.Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].X, s.m_vertices[i].Y);
			//        }
			//        Settings.Log("    shape.Set(vs, %d);\n", s.m_count);
			//    }
			//    break;

			//case ShapeType.chain:
			//    {
			//        ChainShape* s = (ChainShape*)m_shape;
			//        Settings.Log("    ChainShape shape = new ChainShape();\n");
			//        Settings.Log("    Vec2[] vs = new Vec2[%d];\n", s.m_count);
			//        for (int i = 0; i < s.m_count; ++i)
			//        {
			//            Settings.Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].X, s.m_vertices[i].Y);
			//        }
			//        Settings.Log("    shape.CreateChain(vs, %d);\n", s.m_count);
			//        Settings.Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s.m_prevVertex.X, s.m_prevVertex.Y);
			//        Settings.Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s.m_nextVertex.X, s.m_nextVertex.Y);
			//        Settings.Log("    shape.m_hasPrevVertex = (bool)(%d);\n", s.m_hasPrevVertex);
			//        Settings.Log("    shape.m_hasNextVertex = (bool)(%d);\n", s.m_hasNextVertex);
			//    }
			//    break;

			//default:
			//    return;
			//}

			//Settings.Log("\n");
			//Settings.Log("    fd.shape = shape;\n");
			//Settings.Log("\n");
			//Settings.Log("    bodies[%d].CreateFixture(fd);\n", bodyIndex);
		}


		internal Fixture(){
			m_userData = null;
			m_body = null;
			m_next = null;
			m_proxies = new List<FixtureProxy>();
			m_shape = null;
			m_Density = 0.0f;
		}

		// We need separation create/destroy functions from the constructor/destructor because
		// the destructor cannot access the allocator (no destructor arguments allowed by C++).
		internal void Create(Body body, FixtureDef def){
			m_userData = def.UserData;
			m_friction = def.friction;
			m_restitution = def.restitution;

			m_body = body;
			m_next = null;

			m_filter = def.Filter;

			m_isSensor = def.IsSensor;

			m_shape = def.shape.Clone();

			// Reserve proxy space
			int childCount = m_shape.GetChildCount();
			m_proxies = new List<FixtureProxy>();

			m_Density = def.Density;
		}

		protected void Destroy(){
			throw new NotImplementedException();
			//// The proxies must be destroyed before calling this.
			//Utilities.Assert(m_proxyCount == 0);

			//// Free the proxy array.
			//int childCount = m_shape.GetChildCount();
			//allocator.Free(m_proxies, childCount * sizeof(FixtureProxy));
			//m_proxies = null;

			//// Free the child shape.
			//switch (m_shape.m_type)
			//{
			//case ShapeType.circle:
			//    {
			//        CircleShape* s = (CircleShape*)m_shape;
			//        s.~CircleShape();
			//        allocator.Free(s, sizeof(CircleShape));
			//    }
			//    break;

			//case ShapeType.edge:
			//    {
			//        EdgeShape* s = (EdgeShape*)m_shape;
			//        s.~EdgeShape();
			//        allocator.Free(s, sizeof(EdgeShape));
			//    }
			//    break;

			//case ShapeType.polygon:
			//    {
			//        PolygonShape* s = (PolygonShape*)m_shape;
			//        s.~PolygonShape();
			//        allocator.Free(s, sizeof(PolygonShape));
			//    }
			//    break;

			//case ShapeType.chain:
			//    {
			//        ChainShape* s = (ChainShape*)m_shape;
			//        s.~ChainShape();
			//        allocator.Free(s, sizeof(ChainShape));
			//    }
			//    break;

			//default:
			//    Utilities.Assert(false);
			//    break;
			//}

			//m_shape = null;
		}

		// These support body activation/deactivation.
		internal void CreateProxies(BroadPhase broadPhase, Transform xf){ //broadPhase was pointer
			Utilities.Assert(m_proxies.Count() == 0);

			// Create proxies in the broad-phase.
			int m_proxyCount = m_shape.GetChildCount();

			for (int i = 0; i < m_proxyCount; ++i) {
				FixtureProxy proxy = new FixtureProxy();
				m_shape.ComputeAABB(out proxy.aabb, xf, i);
				proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, proxy);
				proxy.fixture = this;
				proxy.childIndex = i;
				m_proxies.Add(proxy);
			}
		}

		protected void DestroyProxies(BroadPhase broadPhase){ //broadphase was pointer
			throw new NotImplementedException();
			//// Destroy proxies in the broad-phase.
			//for (int i = 0; i < m_proxyCount; ++i)
			//{
			//    FixtureProxy* proxy = m_proxies + i;
			//    broadPhase.DestroyProxy(proxy.proxyId);
			//    proxy.proxyId = BroadPhase::e_nullProxy;
			//}

			//m_proxyCount = 0;
		}

		internal void Synchronize(BroadPhase broadPhase, Transform transform1, Transform transform2) { //broadphase was pointer
			if (m_proxies.Count() == 0) {
				return;
			}

			for (int i = 0; i < m_proxies.Count(); ++i) {
				FixtureProxy proxy = m_proxies[i];

				// Compute an AABB that covers the swept shape (may miss some rotation effect).
				AABB aabb1, aab;
				m_shape.ComputeAABB(out aabb1, transform1, proxy.childIndex);
				m_shape.ComputeAABB(out aab, transform2, proxy.childIndex);

				proxy.aabb.Combine(aabb1, aab);

				Vec2 displacement = transform2.p - transform1.p;

				broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
			}
		}	
	}
}
