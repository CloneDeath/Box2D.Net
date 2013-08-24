using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A 3-by-3 matrix. Stored in column-major order.
	public struct b2Mat33
	{
		/// Construct this matrix using columns.
		public b2Mat33(b2Vec3 c1, b2Vec3 c2, b2Vec3 c3)
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
		public b2Vec3 Solve33(b2Vec3 b) {
			float det = Utilities.b2Dot(ex, Utilities.b2Cross(ey, ez));
			if (det != 0.0f) {
				det = 1.0f / det;
			}
			b2Vec3 x;
			x.x = det * Utilities.b2Dot(b, Utilities.b2Cross(ey, ez));
			x.y = det * Utilities.b2Dot(ex, Utilities.b2Cross(b, ez));
			x.z = det * Utilities.b2Dot(ex, Utilities.b2Cross(ey, b));
			return x;
		}

		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases. Solve only the upper
		/// 2-by-2 matrix equation.
		public b2Vec2 Solve22(b2Vec2 b) {
			float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
			float det = a11 * a22 - a12 * a21;
			if (det != 0.0f) {
				det = 1.0f / det;
			}
			b2Vec2 x;
			x.x = det * (a22 * b.x - a12 * b.y);
			x.y = det * (a11 * b.y - a21 * b.x);
			return x;
		}

		/// Get the inverse of this matrix as a 2-by-2.
		/// Returns the zero matrix if singular.
		public void GetInverse22(out b2Mat33 M) {
			float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
			float det = a * d - b * c;
			if (det != 0.0f) {
				det = 1.0f / det;
			}

			M.ex.x = det * d; M.ey.x = -det * b; M.ex.z = 0.0f;
			M.ex.y = -det * c; M.ey.y = det * a; M.ey.z = 0.0f;
			M.ez.x = 0.0f; M.ez.y = 0.0f; M.ez.z = 0.0f;
		}

		/// Get the symmetric inverse of this matrix as a 3-by-3.
		/// Returns the zero matrix if singular.
		public void GetSymInverse33(out b2Mat33 M) {
			float det = Utilities.b2Dot(ex, Utilities.b2Cross(ey, ez));
			if (det != 0.0f) {
				det = 1.0f / det;
			}

			float a11 = ex.x, a12 = ey.x, a13 = ez.x;
			float a22 = ey.y, a23 = ez.y;
			float a33 = ez.z;

			M.ex.x = det * (a22 * a33 - a23 * a23);
			M.ex.y = det * (a13 * a23 - a12 * a33);
			M.ex.z = det * (a12 * a23 - a13 * a22);

			M.ey.x = M.ex.y;
			M.ey.y = det * (a11 * a33 - a13 * a13);
			M.ey.z = det * (a13 * a12 - a11 * a23);

			M.ez.x = M.ex.z;
			M.ez.y = M.ey.z;
			M.ez.z = det * (a11 * a22 - a12 * a12);
		}

		public b2Vec3 ex, ey, ez;
	}
}
