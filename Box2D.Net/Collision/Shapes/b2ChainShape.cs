using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A chain shape is a free form sequence of line segments.
	/// The chain has two-sided collision, so you can use inside and outside collision.
	/// Therefore, you may use any winding order.
	/// Since there may be many vertices, they are allocated using b2Alloc.
	/// Connectivity information is used to create smooth collisions.
	/// WARNING: The chain will not collide properly if there are self-intersections.
	class b2ChainShape : b2Shape {
		public b2ChainShape(){
			m_type = e_chain;
			m_radius = b2Settings.b2_polygonRadius;
			m_vertices = null;
			m_count = 0;
			m_hasPrevVertex = null;
			m_hasNextVertex = null;
		}

		/// The destructor frees the vertices using b2Free.
		public ~b2ChainShape();

		/// Create a loop. This automatically adjusts connectivity.
		/// @param vertices an array of vertices, these are copied
		/// @param count the vertex count
		public void CreateLoop(const b2Vec2* vertices, int count);

		/// Create a chain with isolated end vertices.
		/// @param vertices an array of vertices, these are copied
		/// @param count the vertex count
		public void CreateChain(const b2Vec2* vertices, int count);

		/// Establish connectivity to a vertex that precedes the first vertex.
		/// Don't call this for loops.
		public void SetPrevVertex(const b2Vec2& prevVertex);

		/// Establish connectivity to a vertex that follows the last vertex.
		/// Don't call this for loops.
		public void SetNextVertex(const b2Vec2& nextVertex);

		/// Implement b2Shape. Vertices are cloned using b2Alloc.
		public b2Shape* Clone(b2BlockAllocator* allocator) const;

		/// @see b2Shape::GetChildCount
		public int GetChildCount() const;

		/// Get a child edge.
		public void GetChildEdge(b2EdgeShape* edge, int index) const;

		/// This always return false.
		/// @see b2Shape::TestPoint
		public bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;

		/// Implement b2Shape.
		public bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
						const b2Transform& transform, float childIndex) const;

		/// @see b2Shape::ComputeAABB
		public void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

		/// Chains have zero mass.
		/// @see b2Shape::ComputeMass
		public void ComputeMass(b2MassData* massData, float density) const;

		/// The vertices. Owned by this class.
		public b2Vec2* m_vertices;

		/// The vertex count.
		public int m_count;

		public b2Vec2 m_prevVertex, m_nextVertex;
		public bool m_hasPrevVertex, m_hasNextVertex;
	}
}
