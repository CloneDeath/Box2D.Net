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
			m_type = ShapeType.Edge;
			m_radius = b2Settings.b2_polygonRadius;
			m_vertex0.x = 0.0f;
			m_vertex0.y = 0.0f;
			m_vertex3.x = 0.0f;
			m_vertex3.y = 0.0f;
			m_hasVertex0 = false;
			m_hasVertex3 = false;
		}

		/// Set this as an isolated edge.
		public void Set(b2Vec2 v1, b2Vec2 v2) {
			m_vertex1 = v1;
			m_vertex2 = v2;
			m_hasVertex0 = false;
			m_hasVertex3 = false;
		}

		/// Implement b2Shape.
		public b2Shape Clone(){
			throw new NotImplementedException();
			//void* mem = allocator.Allocate(sizeof(b2EdgeShape));
			//b2EdgeShape* clone = new (mem) b2EdgeShape;
			//*clone = *this;
			//return clone;
		}

		/// @see b2Shape::GetChildCount
		public int GetChildCount() {
			return 1;
		}

		/// @see b2Shape::TestPoint
		public bool TestPoint(b2Transform transform, b2Vec2 p) {
			throw new NotImplementedException();
			//B2_NOT_USED(xf);
			//B2_NOT_USED(p);
			//return false;
		}

		/// Implement b2Shape.
		// p = p1 + t * d
		// v = v1 + s * e
		// p1 + t * d = v1 + s * e
		// s * e - t * d = p1 - v1
		public bool RayCast(out b2RayCastOutput output, b2RayCastInput input,
					b2Transform transform, int childIndex){
			B2_NOT_USED(childIndex);

			// Put the ray into the edge's frame of reference.
			b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
			b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
			b2Vec2 d = p2 - p1;

			b2Vec2 v1 = m_vertex1;
			b2Vec2 v2 = m_vertex2;
			b2Vec2 e = v2 - v1;
			b2Vec2 normal(e.y, -e.x);
			normal.Normalize();

			// q = p1 + t * d
			// dot(normal, q - v1) = 0
			// dot(normal, p1 - v1) + t * dot(normal, d) = 0
			float numerator = b2Dot(normal, v1 - p1);
			float denominator = b2Dot(normal, d);

			if (denominator == 0.0f)
			{
				return false;
			}

			float t = numerator / denominator;
			if (t < 0.0f || input.maxFraction < t)
			{
				return false;
			}

			b2Vec2 q = p1 + t * d;

			// q = v1 + s * r
			// s = dot(q - v1, r) / dot(r, r)
			b2Vec2 r = v2 - v1;
			float rr = b2Dot(r, r);
			if (rr == 0.0f)
			{
				return false;
			}

			float s = b2Dot(q - v1, r) / rr;
			if (s < 0.0f || 1.0f < s)
			{
				return false;
			}

			output.fraction = t;
			if (numerator > 0.0f)
			{
				output.normal = -normal;
			}
			else
			{
				output.normal = normal;
			}
			return true;
		}

		/// @see b2Shape::ComputeAABB
		public void ComputeAABB(out b2AABB aabb, b2Transform transform, int childIndex){
			throw new NotImplementedException();
			//B2_NOT_USED(childIndex);

			//b2Vec2 v1 = b2Mul(xf, m_vertex1);
			//b2Vec2 v2 = b2Mul(xf, m_vertex2);

			//b2Vec2 lower = Math.Min(v1, v2);
			//b2Vec2 upper = b2Max(v1, v2);

			//b2Vec2 r(m_radius, m_radius);
			//aabb.lowerBound = lower - r;
			//aabb.upperBound = upper + r;
		}

		/// @see b2Shape::ComputeMass
		public void ComputeMass(out b2MassData massData, float density) {
			throw new NotImplementedException();
			//B2_NOT_USED(density);

			//massData.mass = 0.0f;
			//massData.center = 0.5f * (m_vertex1 + m_vertex2);
			//massData.I = 0.0f;
		}
	
		/// These are the edge vertices
		public b2Vec2 m_vertex1, m_vertex2;

		/// Optional adjacent vertices. These are used for smooth collision.
		public b2Vec2 m_vertex0, m_vertex3;
		public bool m_hasVertex0, m_hasVertex3;
	}
}
