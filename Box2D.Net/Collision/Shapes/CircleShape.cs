using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A circle shape.
	public class CircleShape : Shape {
		public CircleShape(){
			m_type = ShapeType.Circle;
			m_radius = 0.0f;
			m_p.SetZero();
		}

		/// Implement Shape.
		public override Shape Clone(){
			return (Shape)this.MemberwiseClone();
		}

		/// @see Shape::GetChildCount
		public override int GetChildCount() {
			return 1;
		}
		
		/// Implement Shape.
		public override bool TestPoint(Transform transform, Vec2 p) {
			throw new NotImplementedException();
			//Vec2 center = transform.p + Utilities.Mul(transform.q, m_p);
			//Vec2 d = p - center;
			//return Utilities.Dot(d, d) <= m_radius * m_radius;
		}

		/// Implement Shape.
		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public override bool RayCast(out RayCastOutput output, RayCastInput input,
					Transform transform, int childIndex){
			throw new NotImplementedException();

			//Vec2 position = transform.p + Utilities.Mul(transform.q, m_p);
			//Vec2 s = input.p1 - position;
			//float b = Utilities.Dot(s, s) - m_radius * m_radius;

			//// Solve quadratic equation.
			//Vec2 r = input.p2 - input.p1;
			//float c =  Utilities.Dot(s, r);
			//float rr = Utilities.Dot(r, r);
			//float sigma = c * c - rr * b;

			//// Check for negative discriminant and short segment.
			//if (sigma < 0.0f || rr < Single.Epsilon)
			//{
			//    return false;
			//}

			//// Find the point of intersection of the line with the circle.
			//float a = -(c + (float)Math.Sqrt(sigma));

			//// Is the intersection point on the segment?
			//if (0.0f <= a && a <= input.maxFraction * rr)
			//{
			//    a /= rr;
			//    output.fraction = a;
			//    output.normal = s + a * r;
			//    output.normal.Normalize();
			//    return true;
			//}

			//return false;
		}

		/// @see Shape::ComputeAABB
		public override void ComputeAABB(out AABB aabb, Transform transform, int childIndex) {
			aabb = new AABB();
			Vec2 p = transform.p + Utilities.Mul(transform.q, m_p);
			aabb.lowerBound.Set(p.X - m_radius, p.Y - m_radius);
			aabb.upperBound.Set(p.X + m_radius, p.Y + m_radius);
		}

		/// @see Shape::ComputeMass
		public override void ComputeMass(out MassData massData, float Density) {
			massData = new MassData();
			massData.mass = Density * (float)Math.PI * m_radius * m_radius;
			massData.center = m_p;

			// inertia about the local origin
			massData.I = massData.mass * (0.5f * m_radius * m_radius + Utilities.Dot(m_p, m_p));
		}

		/// Get the supporting vertex index in the given direction.
		public int GetSupport(Vec2 d){
			return 0;
		}

		/// Get the supporting vertex in the given direction.
		public Vec2 GetSupportVertex(Vec2 d){
			return m_p;
		}

		/// Get the vertex count.
		public int GetVertexCount() { return 1; }

		/// Get a vertex by index. Used by Distance.
		public Vec2 GetVertex(int index){
			Utilities.Assert(index == 0);
			return m_p;
		}

		/// Position
		public Vec2 m_p;
	}
}
