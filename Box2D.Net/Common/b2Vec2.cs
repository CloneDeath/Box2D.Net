using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2D column vector.
	public struct b2Vec2 {
		float x, y;

		/// Default constructor does nothing (for performance).
		public b2Vec2() {
			x = 0;
			y = 0;
		}

		/// Construct using coordinates.
		public b2Vec2(float x, float y){
			this.x = x;
			this.y = y;
		}

		/// Set this vector to all zeros.
		public void SetZero() { x = 0.0f; y = 0.0f; }

		/// Set this vector to some specified coordinates.
		public void Set(float x_, float y_) { x = x_; y = y_; }

		/// Negate this vector.
		public b2Vec2 operator -() const { b2Vec2 v; v.Set(-x, -y); return v; }
	
		/// Read from and indexed element.
		public float operator () (int i) const
		{
			return (&x)[i];
		}

		/// Write to an indexed element.
		public float& operator () (int i)
		{
			return (&x)[i];
		}

		/// Add a vector to this vector.
		public void operator += (const b2Vec2& v)
		{
			x += v.x; y += v.y;
		}
	
		/// Subtract a vector from this vector.
		public void operator -= (const b2Vec2& v)
		{
			x -= v.x; y -= v.y;
		}

		/// Multiply this vector by a scalar.
		public void operator *= (float a)
		{
			x *= a; y *= a;
		}

		/// Get the length of this vector (the norm).
		public float Length() const
		{
			return b2Sqrt(x * x + y * y);
		}

		/// Get the length squared. For performance, use this instead of
		/// b2Vec2::Length (if possible).
		public float LengthSquared() const
		{
			return x * x + y * y;
		}

		/// Convert this vector into a unit vector. Returns the length.
		public float Normalize()
		{
			float length = Length();
			if (length < b2_epsilon)
			{
				return 0.0f;
			}
			float invLength = 1.0f / length;
			x *= invLength;
			y *= invLength;

			return length;
		}

		/// Does this vector contain finite coordinates?
		public bool IsValid() const
		{
			return b2IsValid(x) && b2IsValid(y);
		}

		/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
		public b2Vec2 Skew() const
		{
			return b2Vec2(-y, x);
		}
	}
}
