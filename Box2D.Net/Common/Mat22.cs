using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 2-by-2 matrix. Stored in column-major order.
	public struct Mat22
	{
		public Vec2 ex, ey;

		/// Construct this matrix using columns.
		public Mat22(Vec2 c1, Vec2 c2)
		{
			ex = c1;
			ey = c2;
		}

		/// Construct this matrix using scalars.
		public Mat22(float a11, float a12, float a21, float a22)
		{
			ex.X = a11; ex.Y = a21;
			ey.X = a12; ey.Y = a22;
		}

		/// Initialize this matrix using columns.
		public void Set(Vec2 c1, Vec2 c2)
		{
			ex = c1;
			ey = c2;
		}

		/// Set this to the identity matrix.
		public void SetIdentity()
		{
			ex.X = 1.0f; ey.X = 0.0f;
			ex.Y = 0.0f; ey.Y = 1.0f;
		}

		/// Set this matrix to all zeros.
		public void SetZero()
		{
			ex.X = 0.0f; ey.X = 0.0f;
			ex.Y = 0.0f; ey.Y = 0.0f;
		}

		public Mat22 GetInverse()
		{
			float a = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
			Mat22 B = new Mat22();
			float det = a * d - b * c;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			B.ex.X =  det * d;	B.ey.X = -det * b;
			B.ex.Y = -det * c;	B.ey.Y =  det * a;
			return B;
		}

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		public Vec2 Solve(Vec2 b)
		{
			float a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
			float det = a11 * a22 - a12 * a21;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			Vec2 x;
			x.X = det * (a22 * b.X - a12 * b.Y);
			x.Y = det * (a11 * b.Y - a21 * b.X);
			return x;
		}

		public static Mat22 operator +(Mat22 A, Mat22 B)
		{
			return new Mat22(A.ex + B.ex, A.ey + B.ey);
		}
	}
}
