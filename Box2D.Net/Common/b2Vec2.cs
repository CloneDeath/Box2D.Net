﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2D column vector.
	public struct Vec2 {
		public float x, y;

		/// Default constructor does nothing (for performance).
		public Vec2(object ignore) {
			x = 0;
			y = 0;
		}

		/// Construct using coordinates.
		public Vec2(float x, float y){
			this.x = x;
			this.y = y;
		}

		/// Set this vector to all zeros.
		public void SetZero() { x = 0.0f; y = 0.0f; }

		/// Set this vector to some specified coordinates.
		public void Set(float x_, float y_) { x = x_; y = y_; }

		/// Negate this vector.
		public static Vec2 operator-(Vec2 self){
			return new Vec2(-self.x, -self.y);
		}

		public static Vec2 operator +(Vec2 lhs, Vec2 rhs) {
			return new Vec2(lhs.x + rhs.x, lhs.y + rhs.y);
		}

		public static Vec2 operator -(Vec2 lhs, Vec2 rhs) {
			return new Vec2(lhs.x - rhs.x, lhs.y - rhs.y);
		}

		public static Vec2 operator *(float lhs, Vec2 rhs) {
			return new Vec2(rhs.x * lhs, rhs.y * lhs);
		}

		public static Vec2 operator *(Vec2 lhs, float rhs) {
			return new Vec2(rhs * lhs.x, rhs * lhs.y);
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
			return (float)Math.Sqrt((x * x) + (y * y));
		}

		/// Get the length squared. For performance, use this instead of
		/// Vec2::Length (if possible).
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
			return Utilities.IsValid(x) && Utilities.IsValid(y);
		}

		/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
		public Vec2 Skew()
		{
			return new Vec2(-y, x);
		}

		public static bool operator ==(Vec2 a, Vec2 b) {
			return a.x == b.x && a.y == b.y;
		}

		public static bool operator !=(Vec2 lhs, Vec2 rhs) {
			return lhs.x != rhs.x || lhs.y != rhs.y;
		}
	}
}
