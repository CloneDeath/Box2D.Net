using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2-by-2 matrix. Stored in column-major order.
	public struct b2Mat22
	{
		public b2Vec2 ex, ey;

		/// Construct this matrix using columns.
		public b2Mat22(b2Vec2 c1, b2Vec2 c2)
		{
			ex = c1;
			ey = c2;
		}

		/// Construct this matrix using scalars.
		public b2Mat22(float a11, float a12, float a21, float a22)
		{
			ex.x = a11; ex.y = a21;
			ey.x = a12; ey.y = a22;
		}

		/// Initialize this matrix using columns.
		public void Set(b2Vec2 c1, b2Vec2 c2)
		{
			ex = c1;
			ey = c2;
		}

		/// Set this to the identity matrix.
		public void SetIdentity()
		{
			ex.x = 1.0f; ey.x = 0.0f;
			ex.y = 0.0f; ey.y = 1.0f;
		}

		/// Set this matrix to all zeros.
		public void SetZero()
		{
			ex.x = 0.0f; ey.x = 0.0f;
			ex.y = 0.0f; ey.y = 0.0f;
		}

		public b2Mat22 GetInverse()
		{
			float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
			b2Mat22 B = new b2Mat22();
			float det = a * d - b * c;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			B.ex.x =  det * d;	B.ey.x = -det * b;
			B.ex.y = -det * c;	B.ey.y =  det * a;
			return B;
		}

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		public b2Vec2 Solve(b2Vec2 b)
		{
			float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
			float det = a11 * a22 - a12 * a21;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			b2Vec2 x;
			x.x = det * (a22 * b.x - a12 * b.y);
			x.y = det * (a11 * b.y - a21 * b.x);
			return x;
		}

		public static b2Mat22 operator +(b2Mat22 A, b2Mat22 B)
		{
			return new b2Mat22(A.ex + B.ex, A.ey + B.ey);
		}
	}
}
