using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2D column vector with 3 elements.
	public struct Vec3
	{
		/// Construct using coordinates.
		public Vec3(float x, float y, float z) {
			this.X = x;
			this.Y = y;
			this.Z = z;

		}

		/// Set this vector to all zeros.
		public void SetZero() { X = 0.0f; Y = 0.0f; Z = 0.0f; }

		/// Set this vector to some specified coordinates.
		public void Set(float x_, float y_, float z_) { X = x_; Y = y_; Z = z_; }

		/// Negate this vector.
		public static Vec3 operator -(Vec3 other) { 
			Vec3 v = new Vec3(); 
			v.Set(-other.X, -other.Y, -other.Z); 
			return v; 
		}

		public float X, Y, Z;

		public static Vec3 operator * (float s, Vec3 a)
		{
			return new Vec3(s * a.X, s * a.Y, s * a.Z);
		}

		public static Vec3 operator *(Vec3 lhs, float rhs) {
			return new Vec3(lhs.X * rhs, lhs.Y * rhs, lhs.Z * rhs);
		}

		/// Add two vectors component-wise.
		public static Vec3 operator + (Vec3 a, Vec3 b)
		{
			return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
		}

		/// Subtract two vectors component-wise.
		public static Vec3 operator - (Vec3 a, Vec3 b)
		{
			return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
		}
	}
}
