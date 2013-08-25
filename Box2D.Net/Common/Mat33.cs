using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 3-by-3 matrix. Stored in column-major order.
	public struct Mat33
	{
		/// Construct this matrix using columns.
		public Mat33(Vec3 c1, Vec3 c2, Vec3 c3)
		{
			ex = c1;
			ey = c2;
			ez = c3;
		}

		/// Set this matrix to all zeros.
		public void SetZero()
		{
			ex.SetZero();
			ey.SetZero();
			ez.SetZero();
		}

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		public Vec3 Solve33(Vec3 b) {
			float det = Utilities.Dot(ex, Utilities.Cross(ey, ez));
			if (det != 0.0f) {
				det = 1.0f / det;
			}
			Vec3 x;
			x.X = det * Utilities.Dot(b, Utilities.Cross(ey, ez));
			x.Y = det * Utilities.Dot(ex, Utilities.Cross(b, ez));
			x.Z = det * Utilities.Dot(ex, Utilities.Cross(ey, b));
			return x;
		}

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases. Solve only the upper
		/// 2-by-2 matrix equation.
		public Vec2 Solve22(Vec2 b) {
			float a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
			float det = a11 * a22 - a12 * a21;
			if (det != 0.0f) {
				det = 1.0f / det;
			}
			Vec2 x;
			x.X = det * (a22 * b.X - a12 * b.Y);
			x.Y = det * (a11 * b.Y - a21 * b.X);
			return x;
		}

		/// Get the inverse of this matrix as a 2-by-2.
		/// Returns the zero matrix if singular.
		public void GetInverse22(out Mat33 M) {
			float a = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
			float det = a * d - b * c;
			if (det != 0.0f) {
				det = 1.0f / det;
			}

			M.ex.X = det * d; M.ey.X = -det * b; M.ex.Z = 0.0f;
			M.ex.Y = -det * c; M.ey.Y = det * a; M.ey.Z = 0.0f;
			M.ez.X = 0.0f; M.ez.Y = 0.0f; M.ez.Z = 0.0f;
		}

		/// Get the symmetric inverse of this matrix as a 3-by-3.
		/// Returns the zero matrix if singular.
		public void GetSymInverse33(out Mat33 M) {
			float det = Utilities.Dot(ex, Utilities.Cross(ey, ez));
			if (det != 0.0f) {
				det = 1.0f / det;
			}

			float a11 = ex.X, a12 = ey.X, a13 = ez.X;
			float a22 = ey.Y, a23 = ez.Y;
			float a33 = ez.Z;

			M.ex.X = det * (a22 * a33 - a23 * a23);
			M.ex.Y = det * (a13 * a23 - a12 * a33);
			M.ex.Z = det * (a12 * a23 - a13 * a22);

			M.ey.X = M.ex.Y;
			M.ey.Y = det * (a11 * a33 - a13 * a13);
			M.ey.Z = det * (a13 * a12 - a11 * a23);

			M.ez.X = M.ex.Z;
			M.ez.Y = M.ey.Z;
			M.ez.Z = det * (a11 * a22 - a12 * a12);
		}

		public Vec3 ex, ey, ez;
	}
}
