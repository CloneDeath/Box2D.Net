using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A fixture is used to attach a shape to a body for collision detection. A fixture
	/// inherits its transform from its parent. Fixtures hold additional non-geometric data
	/// such as friction, collision filters, etc.
	/// Fixtures are created via b2Body::CreateFixture.
	/// @warning you cannot reuse fixtures.
	public class b2Fixture
	{
		internal float m_density;

		protected b2Fixture m_next; //pointer
		internal b2Body m_body; //pointer

		protected b2Shape m_shape; //pointer

		internal float m_friction;
		internal float m_restitution;

		internal List<b2FixtureProxy> m_proxies; //pointer

		protected b2Filter m_filter;

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
		public b2Shape GetShape(){
			return m_shape;
		}

		/// Set if this fixture is a sensor.
		public void SetSensor(bool sensor){
			if (sensor != m_isSensor)
			{
				m_body.SetAwake(true);
				m_isSensor = sensor;
			}
		}

		/// Is this fixture a sensor (non-solid)?
		/// @return the true if the shape is a sensor.
		public bool IsSensor(){
			return m_isSensor;
		}

		/// Set the contact filtering data. This will not update contacts until the next time
		/// step when either parent body is active and awake.
		/// This automatically calls Refilter.
		public b2Filter Filter {
			get {
				return m_filter;
			}
			set {
				m_filter = value;
				Refilter();
			}
		}

		/// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
		public void Refilter(){
			throw new NotImplementedException();
			//if (m_body == null)
			//{
			//    return;
			//}

			//// Flag associated contacts for filtering.
			//b2ContactEdge* edge = m_body.GetContactList();
			//while (edge)
			//{
			//    b2Contact* contact = edge.contact;
			//    b2Fixture fixtureA = contact.GetFixtureA();
			//    b2Fixture fixtureB = contact.GetFixtureB();
			//    if (fixtureA == this || fixtureB == this)
			//    {
			//        contact.FlagForFiltering();
			//    }

			//    edge = edge.next;
			//}

			//b2World* world = m_body.GetWorld();

			//if (world == null)
			//{
			//    return;
			//}

			//// Touch each proxy so that new pairs may be created
			//b2BroadPhase* broadPhase = &world.m_contactManager.m_broadPhase;
			//for (int i = 0; i < m_proxyCount; ++i)
			//{
			//    broadPhase.TouchProxy(m_proxies[i].proxyId);
			//}
		}

		/// Get the parent body of this fixture. This is null if the fixture is not attached.
		/// @return the parent body.
		public b2Body GetBody(){
			return m_body;
		}

		/// Get the next fixture in the parent body's fixture list.
		/// @return the next shape.
		public b2Fixture GetNext(){
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
		public bool TestPoint(b2Vec2 p){
			return m_shape.TestPoint(m_body.GetTransform(), p);
		}

		/// Cast a ray against this shape.
		/// @param output the ray-cast results.
		/// @param input the ray-cast input parameters.
		public bool RayCast(out b2RayCastOutput output, b2RayCastInput input, int childIndex){
			return m_shape.RayCast(out output, input, m_body.GetTransform(), childIndex);
		}

		/// Get the mass data for this fixture. The mass data is based on the density and
		/// the shape. The rotational inertia is about the shape's origin. This operation
		/// may be expensive.
		public void GetMassData(out b2MassData massData){
			m_shape.ComputeMass(out massData, m_density);
		}

		/// Set the density of this fixture. This will _not_ automatically adjust the mass
		/// of the body. You must call b2Body::ResetMassData to update the body's mass.
		public void SetDensity(float density){
			throw new NotImplementedException();
			//Utilities.Assert(Utilities.IsValid(density) && density >= 0.0f);
			//m_density = density;
		}

		/// Get the density of this fixture.
		public float GetDensity(){
			return m_density;
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
		public b2AABB GetAABB(int childIndex){
			throw new NotImplementedException();
			//Utilities.Assert(0 <= childIndex && childIndex < m_proxyCount);
			//return m_proxies[childIndex].aabb;
		}

		/// Dump this fixture to the log file.
		public void Dump(int bodyIndex){
			throw new NotImplementedException();
			//b2Settings.b2Log("    b2FixtureDef fd = new b2FixtureDef();\n");
			//b2Settings.b2Log("    fd.friction = %.15lef;\n", m_friction);
			//b2Settings.b2Log("    fd.restitution = %.15lef;\n", m_restitution);
			//b2Settings.b2Log("    fd.density = %.15lef;\n", m_density);
			//b2Settings.b2Log("    fd.isSensor = (bool)(%d);\n", m_isSensor);
			//b2Settings.b2Log("    fd.filter.categoryBits = (ushort)(%d);\n", m_filter.categoryBits);
			//b2Settings.b2Log("    fd.filter.maskBits = (ushort)(%d);\n", m_filter.maskBits);
			//b2Settings.b2Log("    fd.filter.groupIndex = (short)(%d);\n", m_filter.groupIndex);

			//switch (m_shape.m_type)
			//{
			//case ShapeType.circle:
			//    {
			//        b2CircleShape* s = (b2CircleShape*)m_shape;
			//        b2Settings.b2Log("    b2CircleShape shape;\n");
			//        b2Settings.b2Log("    shape.m_radius = %.15lef;\n", s.m_radius);
			//        b2Settings.b2Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s.m_p.x, s.m_p.y);
			//    }
			//    break;

			//case ShapeType.edge:
			//    {
			//        b2EdgeShape* s = (b2EdgeShape*)m_shape;
			//        b2Settings.b2Log("    b2EdgeShape shape;\n");
			//        b2Settings.b2Log("    shape.m_radius = %.15lef;\n", s.m_radius);
			//        b2Settings.b2Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s.m_vertex0.x, s.m_vertex0.y);
			//        b2Settings.b2Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s.m_vertex1.x, s.m_vertex1.y);
			//        b2Settings.b2Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s.m_vertex2.x, s.m_vertex2.y);
			//        b2Settings.b2Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s.m_vertex3.x, s.m_vertex3.y);
			//        b2Settings.b2Log("    shape.m_hasVertex0 = (bool)(%d);\n", s.m_hasVertex0);
			//        b2Settings.b2Log("    shape.m_hasVertex3 = (bool)(%d);\n", s.m_hasVertex3);
			//    }
			//    break;

			//case ShapeType.polygon:
			//    {
			//        b2PolygonShape* s = (b2PolygonShape*)m_shape;
			//        b2Settings.b2Log("    b2PolygonShape shape;\n");
			//        b2Settings.b2Log("    b2Vec2[] vs = new b2Vec2[%d];\n", b2Settings.b2_maxPolygonVertices);
			//        for (int i = 0; i < s.m_count; ++i)
			//        {
			//            b2Settings.b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].x, s.m_vertices[i].y);
			//        }
			//        b2Settings.b2Log("    shape.Set(vs, %d);\n", s.m_count);
			//    }
			//    break;

			//case ShapeType.chain:
			//    {
			//        b2ChainShape* s = (b2ChainShape*)m_shape;
			//        b2Settings.b2Log("    b2ChainShape shape = new b2ChainShape();\n");
			//        b2Settings.b2Log("    b2Vec2[] vs = new b2Vec2[%d];\n", s.m_count);
			//        for (int i = 0; i < s.m_count; ++i)
			//        {
			//            b2Settings.b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].x, s.m_vertices[i].y);
			//        }
			//        b2Settings.b2Log("    shape.CreateChain(vs, %d);\n", s.m_count);
			//        b2Settings.b2Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s.m_prevVertex.x, s.m_prevVertex.y);
			//        b2Settings.b2Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s.m_nextVertex.x, s.m_nextVertex.y);
			//        b2Settings.b2Log("    shape.m_hasPrevVertex = (bool)(%d);\n", s.m_hasPrevVertex);
			//        b2Settings.b2Log("    shape.m_hasNextVertex = (bool)(%d);\n", s.m_hasNextVertex);
			//    }
			//    break;

			//default:
			//    return;
			//}

			//b2Settings.b2Log("\n");
			//b2Settings.b2Log("    fd.shape = shape;\n");
			//b2Settings.b2Log("\n");
			//b2Settings.b2Log("    bodies[%d].CreateFixture(fd);\n", bodyIndex);
		}


		internal b2Fixture(){
			m_userData = null;
			m_body = null;
			m_next = null;
			m_proxies = new List<b2FixtureProxy>();
			m_shape = null;
			m_density = 0.0f;
		}

		// We need separation create/destroy functions from the constructor/destructor because
		// the destructor cannot access the allocator (no destructor arguments allowed by C++).
		internal void Create(b2Body body, b2FixtureDef def){
			m_userData = def.userData;
			m_friction = def.friction;
			m_restitution = def.restitution;

			m_body = body;
			m_next = null;

			m_filter = def.filter;

			m_isSensor = def.isSensor;

			m_shape = def.shape.Clone();

			// Reserve proxy space
			int childCount = m_shape.GetChildCount();
			m_proxies = new List<b2FixtureProxy>();

			m_density = def.density;
		}

		protected void Destroy(){
			throw new NotImplementedException();
			//// The proxies must be destroyed before calling this.
			//Utilities.Assert(m_proxyCount == 0);

			//// Free the proxy array.
			//int childCount = m_shape.GetChildCount();
			//allocator.Free(m_proxies, childCount * sizeof(b2FixtureProxy));
			//m_proxies = null;

			//// Free the child shape.
			//switch (m_shape.m_type)
			//{
			//case ShapeType.circle:
			//    {
			//        b2CircleShape* s = (b2CircleShape*)m_shape;
			//        s.~b2CircleShape();
			//        allocator.Free(s, sizeof(b2CircleShape));
			//    }
			//    break;

			//case ShapeType.edge:
			//    {
			//        b2EdgeShape* s = (b2EdgeShape*)m_shape;
			//        s.~b2EdgeShape();
			//        allocator.Free(s, sizeof(b2EdgeShape));
			//    }
			//    break;

			//case ShapeType.polygon:
			//    {
			//        b2PolygonShape* s = (b2PolygonShape*)m_shape;
			//        s.~b2PolygonShape();
			//        allocator.Free(s, sizeof(b2PolygonShape));
			//    }
			//    break;

			//case ShapeType.chain:
			//    {
			//        b2ChainShape* s = (b2ChainShape*)m_shape;
			//        s.~b2ChainShape();
			//        allocator.Free(s, sizeof(b2ChainShape));
			//    }
			//    break;

			//default:
			//    Utilities.Assert(false);
			//    break;
			//}

			//m_shape = null;
		}

		// These support body activation/deactivation.
		internal void CreateProxies(b2BroadPhase broadPhase, b2Transform xf){ //broadPhase was pointer
			Utilities.Assert(m_proxies.Count() == 0);

			// Create proxies in the broad-phase.
			int m_proxyCount = m_shape.GetChildCount();

			for (int i = 0; i < m_proxyCount; ++i) {
				b2FixtureProxy proxy = new b2FixtureProxy();
				m_shape.ComputeAABB(out proxy.aabb, xf, i);
				proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, proxy);
				proxy.fixture = this;
				proxy.childIndex = i;
				m_proxies.Add(proxy);
			}
		}

		protected void DestroyProxies(b2BroadPhase broadPhase){ //broadphase was pointer
			throw new NotImplementedException();
			//// Destroy proxies in the broad-phase.
			//for (int i = 0; i < m_proxyCount; ++i)
			//{
			//    b2FixtureProxy* proxy = m_proxies + i;
			//    broadPhase.DestroyProxy(proxy.proxyId);
			//    proxy.proxyId = b2BroadPhase::e_nullProxy;
			//}

			//m_proxyCount = 0;
		}

		internal void Synchronize(b2BroadPhase broadPhase, b2Transform transform1, b2Transform transform2) { //broadphase was pointer
			if (m_proxies.Count() == 0) {
				return;
			}

			for (int i = 0; i < m_proxies.Count(); ++i) {
				b2FixtureProxy proxy = m_proxies[i];

				// Compute an AABB that covers the swept shape (may miss some rotation effect).
				b2AABB aabb1, aabb2;
				m_shape.ComputeAABB(out aabb1, transform1, proxy.childIndex);
				m_shape.ComputeAABB(out aabb2, transform2, proxy.childIndex);

				proxy.aabb.Combine(aabb1, aabb2);

				b2Vec2 displacement = transform2.p - transform1.p;

				broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
			}
		}	
	}
}
