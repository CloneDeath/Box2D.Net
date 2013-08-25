﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public static partial class Utilities {
		public static void Assert(bool Value) {
			if (!Value) {
				string Message = "Assert failed @ " + Environment.StackTrace.ToString();
				#if DEBUG
					throw new Exception(Message);	
				#else
					Settings.Log(Message);
				#endif
			}
		}

		public static void NotUsed(object value) {
		}

		/// This function is used to ensure that a floating point number is
		/// not a NaN or infinity.
		public static bool IsValid(float x)
		{
			if (float.IsNaN(x))
			{
				return false;
			}
			return float.NegativeInfinity < x && x < float.PositiveInfinity;
		}

		/// Perform the dot product on two vectors.
		public static float Dot(Vec2 a, Vec2 b)
		{
			return a.x * b.x + a.y * b.y;
		}

		/// Perform the cross product on two vectors. In 2D this produces a scalar.
		public static float Cross(Vec2 a, Vec2 b)
		{
			return a.x * b.y - a.y * b.x;
		}

		/// Perform the cross product on a vector and a scalar. In 2D this produces
		/// a vector.
		public static Vec2 Cross(Vec2 a, float s)
		{
			return new Vec2(s * a.y, -s * a.x);
		}

		/// Perform the cross product on a scalar and a vector. In 2D this produces
		/// a vector.
		public static Vec2 Cross(float s, Vec2 a)
		{
			return new Vec2(-s * a.y, s * a.x);
		}

		/// Multiply a matrix times a vector. If a rotation matrix is provided,
		/// then this transforms the vector from one frame to another.
		public static Vec2 Mul(Mat22 A, Vec2 v)
		{
			return new Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
		}

		/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
		/// then this transforms the vector from one frame to another (inverse transform).
		public static Vec2 MulT(Mat22 A, Vec2 v)
		{
			return new Vec2(Dot(v, A.ex), Dot(v, A.ey));
		}

		public static float Distance(Vec2 a, Vec2 b)
		{
			Vec2 c = a - b;
			return c.Length();
		}

		public static float DistanceSquared(Vec2 a, Vec2 b)
		{
			Vec2 c = a - b;
			return Utilities.Dot(c, c);
		}

		/// Perform the dot product on two vectors.
		public static float Dot(Vec3 a, Vec3 b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}

		/// Perform the cross product on two vectors.
		public static Vec3 Cross(Vec3 a, Vec3 b)
		{
			return new Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}

		

		// A * B
		public static Mat22 Mul(Mat22 A, Mat22 B)
		{
			return new Mat22(Mul(A, B.ex), Mul(A, B.ey));
		}

		// A^T * B
		public static Mat22 MulT(Mat22 A, Mat22 B)
		{
			Vec2 c1 = new Vec2(Dot(A.ex, B.ex), Dot(A.ey, B.ex));
			Vec2 c2 = new Vec2(Dot(A.ex, B.ey), Dot(A.ey, B.ey));
			return new Mat22(c1, c2);
		}

		/// Multiply a matrix times a vector.
		public static Vec3 Mul(Mat33 A, Vec3 v)
		{
			return v.x * A.ex + v.y * A.ey + v.z * A.ez;
		}

		/// Multiply a matrix times a vector.
		public static Vec2 Mul22(Mat33 A, Vec2 v)
		{
			return new Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
		}

		/// Multiply two rotations: q * r
		public static  Rot Mul(Rot q, Rot r)
		{
			// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
			// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
			// s = qs * rc + qc * rs
			// c = qc * rc - qs * rs
			Rot qr;
			qr.s = q.s * r.c + q.c * r.s;
			qr.c = q.c * r.c - q.s * r.s;
			return qr;
		}

		/// Transpose multiply two rotations: qT * r
		public static  Rot MulT(Rot q, Rot r)
		{
			// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
			// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
			// s = qc * rs - qs * rc
			// c = qc * rc + qs * rs
			Rot qr;
			qr.s = q.c * r.s - q.s * r.c;
			qr.c = q.c * r.c + q.s * r.s;
			return qr;
		}

		/// Rotate a vector
		public static  Vec2 Mul(Rot q, Vec2 v)
		{
			return new Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
		}

		/// Inverse rotate a vector
		public static  Vec2 MulT(Rot q, Vec2 v)
		{
			return new Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
		}

		public static  Vec2 Mul(Transform T, Vec2 v)
		{
			float x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
			float y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

			return new Vec2(x, y);
		}

		public static Vec2 MulT(Transform T, Vec2 v)
		{
			float px = v.x - T.p.x;
			float py = v.y - T.p.y;
			float x = (T.q.c * px + T.q.s * py);
			float y = (-T.q.s * px + T.q.c * py);

			return new Vec2(x, y);
		}

		// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
		//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
		public static Transform Mul( Transform A,  Transform B)
		{
			Transform C;
			C.q = Utilities.Mul(A.q, B.q);
			C.p = Utilities.Mul(A.q, B.p) + A.p;
			return C;
		}

		// v2 = A.q' * (B.q * v1 + B.p - A.p)
		//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
		public static Transform MulT( Transform A,  Transform B)
		{
			Transform C = new Transform();
			C.q = Utilities.MulT(A.q, B.q);
			C.p = Utilities.MulT(A.q, B.p - A.p);
			return C;
		}

		public static Vec2 Min(Vec2 a, Vec2 b)
		{
			return new Vec2(Math.Min(a.x, b.x), Math.Min(a.y, b.y));
		}
		public static Vec2 Max(Vec2 a, Vec2 b)
		{
			return new Vec2(Math.Max(a.x, b.x), Math.Max(a.y, b.y));
		}

		internal static float Clamp(float val, float lo, float hi) {
			return Math.Max(lo, Math.Min(hi, val));
		}

		public static Vec2 Clamp(Vec2 a, Vec2 low, Vec2 high)
		{
			return Max(low, Min(a, high));
		}

		/// "Next Largest Power of 2
		/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
		/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
		/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
		/// largest power of 2. For a 32-bit value:"
		public static uint NextPowerOfTwo(uint x)
		{
			x |= (x >> 1);
			x |= (x >> 2);
			x |= (x >> 4);
			x |= (x >> 8);
			x |= (x >> 16);
			return x + 1;
		}

		public static bool IsPowerOfTwo(uint x)
		{
			bool result = x > 0 && (x & (x - 1)) == 0;
			return result;
		}

		public static Vec2 Abs(Vec2 a)
		{
			return new Vec2(Math.Abs(a.x), Math.Abs(a.y));
		}

		public static Mat22 Abs(Mat22 A)
		{
			return new Mat22(Abs(A.ex), Abs(A.ey));
		}

		/// This is a approximate yet fast inverse square-root.
		public static float InvSqrt(float x)
		{
			return 1 / (float)Math.Sqrt(x);
		}
	}
}
