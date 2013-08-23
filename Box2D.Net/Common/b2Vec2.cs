using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2D column vector.
	public struct b2Vec2 {
		public float x, y;

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
		public static b2Vec2 operator-(b2Vec2 self){
			return new b2Vec2(-self.x, -self.y);
		}

		public static b2Vec2 operator +(b2Vec2 lhs, b2Vec2 rhs) {
			return new b2Vec2(lhs.x + rhs.x, lhs.y + rhs.y);
		}

		public static b2Vec2 operator -(b2Vec2 lhs, b2Vec2 rhs) {
			return new b2Vec2(lhs.x - rhs.x, lhs.y - rhs.y);
		}
	
		/// Read from and indexed element.
		public float this[int i]
		{
			get {
				if (i > 1) throw new IndexOutOfRangeException();
				return (i == 0)? x: y;
			}
			set {
				if (i == 0){
					x = value;
				} else {
					y = value;
				}
			}
		}

		/// Get the length of this vector (the norm).
		public float Length()
		{
			throw new NotImplementedException();
			//return b2Sqrt(x * x + y * y);
		}

		/// Get the length squared. For performance, use this instead of
		/// b2Vec2::Length (if possible).
		public float LengthSquared()
		{
			return (x * x) + (y * y);
		}

		/// Convert this vector into a unit vector. Returns the length.
		public float Normalize()
		{
			float length = Length();
			if (length < Single.Epsilon)
			{
				return 0.0f;
			}
			float invLength = 1.0f / length;
			x *= invLength;
			y *= invLength;

			return length;
		}

		/// Does this vector contain finite coordinates?
		public bool IsValid()
		{
			throw new NotImplementedException();
			//return b2IsValid(x) && b2IsValid(y);
		}

		/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
		public b2Vec2 Skew()
		{
			return new b2Vec2(-y, x);
		}
	}
}
