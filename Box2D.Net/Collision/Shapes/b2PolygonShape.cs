using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A convex polygon. It is assumed that the interior of the polygon is to
	/// the left of each edge.
	/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
	/// In most cases you should not need many vertices for a convex polygon.
	class b2PolygonShape : b2Shape {
		public b2PolygonShape(){
			m_type = e_polygon;
			m_radius = b2_polygonRadius;
			m_count = 0;
			m_centroid.SetZero();
		}

		/// Implement b2Shape.
		public b2Shape* Clone(b2BlockAllocator* allocator) const;

		/// @see b2Shape::GetChildCount
		public int GetChildCount() const;

		/// Create a convex hull from the given array of local points.
		/// The count must be in the range [3, b2_maxPolygonVertices].
		/// @warning the points may be re-ordered, even if they form a convex polygon
		/// @warning collinear points are handled but not removed. Collinear points
		/// may lead to poor stacking behavior.
		public void Set(const b2Vec2* points, int count);

		/// Build vertices to represent an axis-aligned box centered on the local origin.
		/// @param hx the half-width.
		/// @param hy the half-height.
		public void SetAsBox(float hx, float hy);

		/// Build vertices to represent an oriented box.
		/// @param hx the half-width.
		/// @param hy the half-height.
		/// @param center the center of the box in local coordinates.
		/// @param angle the rotation of the box in local coordinates.
		public void SetAsBox(float hx, float hy, const b2Vec2& center, float angle);

		/// @see b2Shape::TestPoint
		public bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;

		/// Implement b2Shape.
		public bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
						const b2Transform& transform, int childIndex) const;

		/// @see b2Shape::ComputeAABB
		public void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

		/// @see b2Shape::ComputeMass
		public void ComputeMass(b2MassData* massData, float density) const;

		/// Get the vertex count.
		public int GetVertexCount() const { return m_count; }

		/// Get a vertex by index.
		public const b2Vec2& GetVertex(int index) const{
			b2Assert(0 <= index && index < m_count);
			return m_vertices[index];
		}

		/// Validate convexity. This is a very time consuming operation.
		/// @returns true if valid
		public bool Validate() const;

		public b2Vec2 m_centroid;
		public b2Vec2 m_vertices[b2_maxPolygonVertices];
		public b2Vec2 m_normals[b2_maxPolygonVertices];
		public int m_count;
	}
}
