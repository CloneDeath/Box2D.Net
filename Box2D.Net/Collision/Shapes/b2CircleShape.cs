using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A circle shape.
	class b2CircleShape : b2Shape {
		public b2CircleShape(){
			m_type = e_circle;
			m_radius = 0.0f;
			m_p.SetZero();
		}

		/// Implement b2Shape.
		public b2Shape* Clone(b2BlockAllocator* allocator) const;

		/// @see b2Shape::GetChildCount
		public int GetChildCount() const;

		/// Implement b2Shape.
		public bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;

		/// Implement b2Shape.
		public bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const b2Transform& transform, int childIndex) const;

		/// @see b2Shape::ComputeAABB
		public void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

		/// @see b2Shape::ComputeMass
		public void ComputeMass(b2MassData* massData, float density) const;

		/// Get the supporting vertex index in the given direction.
		public int GetSupport(const b2Vec2& d) const{
			B2_NOT_USED(d);
			return 0;
		}

		/// Get the supporting vertex in the given direction.
		public const b2Vec2& GetSupportVertex(const b2Vec2& d) const{
			B2_NOT_USED(d);
			return m_p;
		}

		/// Get the vertex count.
		public int GetVertexCount() const { return 1; }

		/// Get a vertex by index. Used by b2Distance.
		public const b2Vec2& GetVertex(int index) const{
			B2_NOT_USED(index);
			b2Assert(index == 0);
			return m_p;
		}

		/// Position
		public b2Vec2 m_p;
	}
}
