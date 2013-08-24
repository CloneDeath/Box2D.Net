using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2D column vector with 3 elements.
	public struct b2Vec3
	{
		/// Construct using coordinates.
		public b2Vec3(float x, float y, float z) {
			this.x = x;
			this.y = y;
			this.z = z;

		}

		/// Set this vector to all zeros.
		public void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

		/// Set this vector to some specified coordinates.
		public void Set(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

		/// Negate this vector.
		public static b2Vec3 operator -(b2Vec3 other) { 
			b2Vec3 v = new b2Vec3(); 
			v.Set(-other.x, -other.y, -other.z); 
			return v; 
		}

		public float x, y, z;

		public static b2Vec3 operator * (float s, b2Vec3 a)
		{
			return new b2Vec3(s * a.x, s * a.y, s * a.z);
		}

		/// Add two vectors component-wise.
		public static b2Vec3 operator + (b2Vec3 a, b2Vec3 b)
		{
			return new b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
		}

		/// Subtract two vectors component-wise.
		public static b2Vec3 operator - (b2Vec3 a, b2Vec3 b)
		{
			return new b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
		}
	}
}
