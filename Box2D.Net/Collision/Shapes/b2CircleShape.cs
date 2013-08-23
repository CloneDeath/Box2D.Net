using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A circle shape.
	public class b2CircleShape : b2Shape {
		public b2CircleShape(){
			m_type = ShapeType.Circle;
			m_radius = 0.0f;
			m_p.SetZero();
		}

		/// Implement b2Shape.
		public override b2Shape Clone(){
			throw new NotImplementedException();
			//void* mem = allocator.Allocate(sizeof(b2CircleShape));
			//b2CircleShape* clone = new (mem) b2CircleShape;
			//*clone = *this;
			//return clone;
		}

		/// @see b2Shape::GetChildCount
		public override int GetChildCount() {
			return 1;
		}
		
		/// Implement b2Shape.
		public override bool TestPoint(b2Transform transform, b2Vec2 p) {
			throw new NotImplementedException();
			//b2Vec2 center = transform.p + Utilities.b2Mul(transform.q, m_p);
			//b2Vec2 d = p - center;
			//return Utilities.b2Dot(d, d) <= m_radius * m_radius;
		}

		/// Implement b2Shape.
		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public override bool RayCast(out b2RayCastOutput output, b2RayCastInput input,
					b2Transform transform, int childIndex){
			throw new NotImplementedException();
			//B2_NOT_USED(childIndex);

			//b2Vec2 position = transform.p + Utilities.b2Mul(transform.q, m_p);
			//b2Vec2 s = input.p1 - position;
			//float b = Utilities.b2Dot(s, s) - m_radius * m_radius;

			//// Solve quadratic equation.
			//b2Vec2 r = input.p2 - input.p1;
			//float c =  Utilities.b2Dot(s, r);
			//float rr = Utilities.b2Dot(r, r);
			//float sigma = c * c - rr * b;

			//// Check for negative discriminant and short segment.
			//if (sigma < 0.0f || rr < Single.Epsilon)
			//{
			//    return false;
			//}

			//// Find the point of intersection of the line with the circle.
			//float a = -(c + b2Sqrt(sigma));

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

		/// @see b2Shape::ComputeAABB
		public override void ComputeAABB(out b2AABB aabb, b2Transform transform, int childIndex) {
			throw new NotImplementedException();
			//B2_NOT_USED(childIndex);

			//b2Vec2 p = transform.p + Utilities.b2Mul(transform.q, m_p);
			//aabb.lowerBound.Set(p.x - m_radius, p.y - m_radius);
			//aabb.upperBound.Set(p.x + m_radius, p.y + m_radius);
		}

		/// @see b2Shape::ComputeMass
		public override void ComputeMass(out b2MassData massData, float density) {
			throw new NotImplementedException();
			//massData.mass = density * Math.PI * m_radius * m_radius;
			//massData.center = m_p;

			//// inertia about the local origin
			//massData.I = massData.mass * (0.5f * m_radius * m_radius + Utilities.b2Dot(m_p, m_p));
		}

		/// Get the supporting vertex index in the given direction.
		public int GetSupport(b2Vec2 d){
			throw new NotImplementedException();
			//B2_NOT_USED(d);
			//return 0;
		}

		/// Get the supporting vertex in the given direction.
		public b2Vec2 GetSupportVertex(b2Vec2 d){
			throw new NotImplementedException();
			//B2_NOT_USED(d);
			//return m_p;
		}

		/// Get the vertex count.
		public int GetVertexCount() { return 1; }

		/// Get a vertex by index. Used by b2Distance.
		public b2Vec2 GetVertex(int index){
			throw new NotImplementedException();
			//B2_NOT_USED(index);
			//Utilities.Assert(index == 0);
			//return m_p;
		}

		/// Position
		public b2Vec2 m_p;
	}
}
