using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A line segment (edge) shape. These can be connected in chains or loops
	/// to other edge shapes. The connectivity information is used to ensure
	/// correct contact normals.
	class b2EdgeShape : b2Shape {
		public b2EdgeShape(){
			m_type = e_edge;
			m_radius = b2_polygonRadius;
			m_vertex0.x = 0.0f;
			m_vertex0.y = 0.0f;
			m_vertex3.x = 0.0f;
			m_vertex3.y = 0.0f;
			m_hasVertex0 = false;
			m_hasVertex3 = false;
		}

		/// Set this as an isolated edge.
		public void Set(const b2Vec2& v1, const b2Vec2& v2);

		/// Implement b2Shape.
		public b2Shape* Clone(b2BlockAllocator* allocator) const;

		/// @see b2Shape::GetChildCount
		public int GetChildCount() const;

		/// @see b2Shape::TestPoint
		public bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;

		/// Implement b2Shape.
		public bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const b2Transform& transform, int childIndex) const;

		/// @see b2Shape::ComputeAABB
		public void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

		/// @see b2Shape::ComputeMass
		public void ComputeMass(b2MassData* massData, float density) const;
	
		/// These are the edge vertices
		public b2Vec2 m_vertex1, m_vertex2;

		/// Optional adjacent vertices. These are used for smooth collision.
		public b2Vec2 m_vertex0, m_vertex3;
		public bool m_hasVertex0, m_hasVertex3;
	}
}
