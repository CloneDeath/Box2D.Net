using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A line segment (edge) shape. These can be connected in chains or loops
	/// to other edge shapes. The connectivity information is used to ensure
	/// correct contact normals.
	public class EdgeShape : Shape {
		public EdgeShape(){
			m_type = ShapeType.Edge;
			m_radius = Settings._polygonRadius;
			m_vertex0.x = 0.0f;
			m_vertex0.y = 0.0f;
			m_vertex3.x = 0.0f;
			m_vertex3.y = 0.0f;
			m_hasVertex0 = false;
			m_hasVertex3 = false;
		}

		/// Set this as an isolated edge.
		public void Set(Vec2 v1, Vec2 v2) {
			m_vertex1 = v1;
			m_vertex2 = v2;
			m_hasVertex0 = false;
			m_hasVertex3 = false;
		}

		/// Implement Shape.
		public override Shape Clone(){
			return (Shape)this.MemberwiseClone();
		}

		/// @see Shape::GetChildCount
		public override int GetChildCount() {
			return 1;
		}

		/// @see Shape::TestPoint
		public override bool TestPoint(Transform transform, Vec2 p) {
			return false;
		}

		/// Implement Shape.
		// p = p1 + t * d
		// v = v1 + s * e
		// p1 + t * d = v1 + s * e
		// s * e - t * d = p1 - v1
		public override bool RayCast(out RayCastOutput output, RayCastInput input,
					Transform transform, int childIndex){
			throw new NotImplementedException();

			//// Put the ray into the edge's frame of reference.
			//Vec2 p1 = Utilities.MulT(xf.q, input.p1 - xf.p);
			//Vec2 p2 = Utilities.MulT(xf.q, input.p2 - xf.p);
			//Vec2 d = p2 - p1;

			//Vec2 v1 = m_vertex1;
			//Vec2 v2 = m_vertex2;
			//Vec2 e = v2 - v1;
			//Vec2 normal(e.y, -e.x);
			//normal.Normalize();

			//// q = p1 + t * d
			//// dot(normal, q - v1) = 0
			//// dot(normal, p1 - v1) + t * dot(normal, d) = 0
			//float numerator = Utilities.Dot(normal, v1 - p1);
			//float denominator = Utilities.Dot(normal, d);

			//if (denominator == 0.0f)
			//{
			//    return false;
			//}

			//float t = numerator / denominator;
			//if (t < 0.0f || input.maxFraction < t)
			//{
			//    return false;
			//}

			//Vec2 q = p1 + t * d;

			//// q = v1 + s * r
			//// s = dot(q - v1, r) / dot(r, r)
			//Vec2 r = v2 - v1;
			//float rr = Utilities.Dot(r, r);
			//if (rr == 0.0f)
			//{
			//    return false;
			//}

			//float s = Utilities.Dot(q - v1, r) / rr;
			//if (s < 0.0f || 1.0f < s)
			//{
			//    return false;
			//}

			//output.fraction = t;
			//if (numerator > 0.0f)
			//{
			//    output.normal = -normal;
			//}
			//else
			//{
			//    output.normal = normal;
			//}
			//return true;
		}

		/// @see Shape::ComputeAABB
		public override void ComputeAABB(out AABB aabb, Transform xf, int childIndex) {
			Vec2 v1 = Utilities.Mul(xf, m_vertex1);
			Vec2 v2 = Utilities.Mul(xf, m_vertex2);

			Vec2 lower = Utilities.Min(v1, v2);
			Vec2 upper = Utilities.Max(v1, v2);

			Vec2 r = new Vec2(m_radius, m_radius);
			aabb.lowerBound = lower - r;
			aabb.upperBound = upper + r;
		}

		/// @see Shape::ComputeMass
		public override void ComputeMass(out MassData massData, float density) {
			throw new NotImplementedException();

			//massData.mass = 0.0f;
			//massData.center = 0.5f * (m_vertex1 + m_vertex2);
			//massData.I = 0.0f;
		}
	
		/// These are the edge vertices
		public Vec2 m_vertex1, m_vertex2;

		/// Optional adjacent vertices. These are used for smooth collision.
		public Vec2 m_vertex0, m_vertex3;
		public bool m_hasVertex0, m_hasVertex3;
	}
}
