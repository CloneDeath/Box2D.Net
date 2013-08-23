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
		public b2Shape Clone(){
			void* mem = allocator->Allocate(sizeof(b2CircleShape));
			b2CircleShape* clone = new (mem) b2CircleShape;
			*clone = *this;
			return clone;
		}

		/// @see b2Shape::GetChildCount
		public int GetChildCount(){
			return 1;
		}
		
		/// Implement b2Shape.
		public bool TestPoint(b2Transform transform, b2Vec2 p){
			b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
			b2Vec2 d = p - center;
			return b2Dot(d, d) <= m_radius * m_radius;
		}

		/// Implement b2Shape.
		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public bool RayCast(out b2RayCastOutput output, b2RayCastInput input,
					b2Transform transform, int childIndex){
			B2_NOT_USED(childIndex);

			b2Vec2 position = transform.p + b2Mul(transform.q, m_p);
			b2Vec2 s = input.p1 - position;
			float b = b2Dot(s, s) - m_radius * m_radius;

			// Solve quadratic equation.
			b2Vec2 r = input.p2 - input.p1;
			float c =  b2Dot(s, r);
			float rr = b2Dot(r, r);
			float sigma = c * c - rr * b;

			// Check for negative discriminant and short segment.
			if (sigma < 0.0f || rr < b2_epsilon)
			{
				return false;
			}

			// Find the point of intersection of the line with the circle.
			float a = -(c + b2Sqrt(sigma));

			// Is the intersection point on the segment?
			if (0.0f <= a && a <= input.maxFraction * rr)
			{
				a /= rr;
				output->fraction = a;
				output->normal = s + a * r;
				output->normal.Normalize();
				return true;
			}

			return false;
		}

		/// @see b2Shape::ComputeAABB
		public void ComputeAABB(out b2AABB aabb, b2Transform transform, int childIndex){
			B2_NOT_USED(childIndex);

			b2Vec2 p = transform.p + b2Mul(transform.q, m_p);
			aabb->lowerBound.Set(p.x - m_radius, p.y - m_radius);
			aabb->upperBound.Set(p.x + m_radius, p.y + m_radius);
		}

		/// @see b2Shape::ComputeMass
		public void ComputeMass(out b2MassData massData, float density){
			massData->mass = density * Math.PI * m_radius * m_radius;
			massData->center = m_p;

			// inertia about the local origin
			massData->I = massData->mass * (0.5f * m_radius * m_radius + b2Dot(m_p, m_p));
		}

		/// Get the supporting vertex index in the given direction.
		public int GetSupport(b2Vec2 d){
			B2_NOT_USED(d);
			return 0;
		}

		/// Get the supporting vertex in the given direction.
		public b2Vec2 GetSupportVertex(b2Vec2 d){
			B2_NOT_USED(d);
			return m_p;
		}

		/// Get the vertex count.
		public int GetVertexCount() { return 1; }

		/// Get a vertex by index. Used by b2Distance.
		public b2Vec2 GetVertex(int index){
			B2_NOT_USED(index);
			b2Assert(index == 0);
			return m_p;
		}

		/// Position
		public b2Vec2 m_p;
	}
}
