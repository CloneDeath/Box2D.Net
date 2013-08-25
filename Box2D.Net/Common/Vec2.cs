using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2D column vector.
	public struct Vec2 {
		public float X, Y;

		/// Construct using coordinates.
		public Vec2(float x, float y){
			this.X = x;
			this.Y = y;
		}

		/// Set this vector to all zeros.
		public void SetZero() { X = 0.0f; Y = 0.0f; }

		/// Set this vector to some specified coordinates.
		public void Set(float x_, float y_) { X = x_; Y = y_; }

		/// Negate this vector.
		public static Vec2 operator-(Vec2 self){
			return new Vec2(-self.X, -self.Y);
		}

		public static Vec2 operator +(Vec2 lhs, Vec2 rhs) {
			return new Vec2(lhs.X + rhs.X, lhs.Y + rhs.Y);
		}

		public static Vec2 operator -(Vec2 lhs, Vec2 rhs) {
			return new Vec2(lhs.X - rhs.X, lhs.Y - rhs.Y);
		}

		public static Vec2 operator *(float lhs, Vec2 rhs) {
			return new Vec2(rhs.X * lhs, rhs.Y * lhs);
		}

		public static Vec2 operator *(Vec2 lhs, float rhs) {
			return new Vec2(rhs * lhs.X, rhs * lhs.Y);
		}
	
		/// Read from and indexed element.
		public float this[int i]
		{
			get {
				if (i > 1) throw new IndexOutOfRangeException();
				return (i == 0)? X: Y;
			}
			set {
				if (i == 0){
					X = value;
				} else {
					Y = value;
				}
			}
		}

		/// Get the length of this vector (the norm).
		public float Length()
		{
			return (float)Math.Sqrt((X * X) + (Y * Y));
		}

		/// Get the length squared. For performance, use this instead of
		/// Vec2::Length (if possible).
		public float LengthSquared()
		{
			return (X * X) + (Y * Y);
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
			X *= invLength;
			Y *= invLength;

			return length;
		}

		/// Does this vector contain finite coordinates?
		public bool IsValid()
		{
			return Utilities.IsValid(X) && Utilities.IsValid(Y);
		}

		/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
		public Vec2 Skew()
		{
			return new Vec2(-Y, X);
		}

		public static bool operator ==(Vec2 a, Vec2 b) {
			return a.X == b.X && a.Y == b.Y;
		}

		public static bool operator !=(Vec2 lhs, Vec2 rhs) {
			return lhs.X != rhs.X || lhs.Y != rhs.Y;
		}
	}
}
