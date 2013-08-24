using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2Simplex
	{
		public void ReadCache(b2SimplexCache cache,
					   b2DistanceProxy proxyA, b2Transform transformA,
					   b2DistanceProxy proxyB, b2Transform transformB)
		{
			Utilities.Assert(cache.count <= 3);
		
			// Copy data from cache.
			m_count = cache.count;
			b2SimplexVertex[] vertices = this.verticies;
			for (int i = 0; i < m_count; ++i)
			{
			    b2SimplexVertex v = vertices[i];
			    v.indexA = cache.indexA[i];
			    v.indexB = cache.indexB[i];
			    b2Vec2 wALocal = proxyA.GetVertex(v.indexA);
			    b2Vec2 wBLocal = proxyB.GetVertex(v.indexB);
			    v.wA = Utilities.b2Mul(transformA, wALocal);
			    v.wB = Utilities.b2Mul(transformB, wBLocal);
			    v.w = v.wB - v.wA;
			    v.a = 0.0f;
			}

			// Compute the new simplex metric, if it is substantially different than
			// old metric then flush the simplex.
			if (m_count > 1)
			{
			    float metric1 = cache.metric;
			    float metric2 = GetMetric();
			    if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Single.Epsilon)
			    {
			        // Reset the simplex.
			        m_count = 0;
			    }
			}

			// If the cache is empty or invalid ...
			if (m_count == 0)
			{
			    b2SimplexVertex v = vertices[0];
			    v.indexA = 0;
			    v.indexB = 0;
			    b2Vec2 wALocal = proxyA.GetVertex(0);
			    b2Vec2 wBLocal = proxyB.GetVertex(0);
			    v.wA = Utilities.b2Mul(transformA, wALocal);
			    v.wB = Utilities.b2Mul(transformB, wBLocal);
			    v.w = v.wB - v.wA;
			    v.a = 1.0f;
			    m_count = 1;
			}
		}

		public void WriteCache(b2SimplexCache cache)
		{
			cache.metric = GetMetric();
			cache.count = (ushort)m_count;
			b2SimplexVertex[] vertices = this.verticies;
			for (int i = 0; i < m_count; ++i)
			{
			    cache.indexA[i] = (byte)(vertices[i].indexA);
			    cache.indexB[i] = (byte)(vertices[i].indexB);
			}
		}

		public b2Vec2 GetSearchDirection()
		{
			switch (m_count) {
				case 1:
					return -m_v1.w;

				case 2: {
						b2Vec2 e12 = m_v2.w - m_v1.w;
						float sgn = Utilities.b2Cross(e12, -m_v1.w);
						if (sgn > 0.0f) {
							// Origin is left of e12.
							return Utilities.b2Cross(1.0f, e12);
						} else {
							// Origin is right of e12.
							return Utilities.b2Cross(e12, 1.0f);
						}
					}

				default:
					Utilities.Assert(false);
					return new b2Vec2(0, 0);
			}
		}

		public b2Vec2 GetClosestPoint()
		{
			switch (m_count) {
				case 0:
					Utilities.Assert(false);
					return new b2Vec2(0, 0);

				case 1:
					return m_v1.w;

				case 2:
					return m_v1.a * m_v1.w + m_v2.a * m_v2.w;

				case 3:
					return new b2Vec2(0, 0);

				default:
					Utilities.Assert(false);
					return new b2Vec2(0, 0);
			}
		}

		public void GetWitnessPoints(out b2Vec2 pA, out b2Vec2 pB)
		{
			pA = new b2Vec2();
			pB = new b2Vec2();
			switch (m_count) {
				case 0:
					Utilities.Assert(false);
					break;

				case 1:
					pA = m_v1.wA;
					pB = m_v1.wB;
					break;

				case 2:
					pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
					pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
					break;

				case 3:
					pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
					pB = pA;
					break;

				default:
					Utilities.Assert(false);
					break;
			}
		}

		public float GetMetric()
		{
			switch (m_count) {
				case 0:
					Utilities.Assert(false);
					return 0.0f;

				case 1:
					return 0.0f;

				case 2:
					return Utilities.b2Distance(m_v1.w, m_v2.w);

				case 3:
					return Utilities.b2Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);

				default:
					Utilities.Assert(false);
					return 0.0f;
			}
		}

		// Solve a line segment using barycentric coordinates.
		//
		// p = a1 * w1 + a2 * w2
		// a1 + a2 = 1
		//
		// The vector from the origin to the closest point on the line is
		// perpendicular to the line.
		// e12 = w2 - w1
		// dot(p, e) = 0
		// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
		//
		// 2-by-2 linear system
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		//
		// Define
		// d12_1 =  dot(w2, e12)
		// d12_2 = -dot(w1, e12)
		// d12 = d12_1 + d12_2
		//
		// Solution
		// a1 = d12_1 / d12
		// a2 = d12_2 / d12
		public void Solve2() {
			b2Vec2 w1 = m_v1.w;
			b2Vec2 w2 = m_v2.w;
			b2Vec2 e12 = w2 - w1;

			// w1 region
			float d12_2 = -Utilities.b2Dot(w1, e12);
			if (d12_2 <= 0.0f) {
				// a2 <= 0, so we clamp it to 0
				m_v1.a = 1.0f;
				m_count = 1;
				return;
			}

			// w2 region
			float d12_1 = Utilities.b2Dot(w2, e12);
			if (d12_1 <= 0.0f) {
				// a1 <= 0, so we clamp it to 0
				m_v2.a = 1.0f;
				m_count = 1;
				m_v1 = m_v2;
				return;
			}

			// Must be in e12 region.
			float inv_d12 = 1.0f / (d12_1 + d12_2);
			m_v1.a = d12_1 * inv_d12;
			m_v2.a = d12_2 * inv_d12;
			m_count = 2;
		}

		// Possible regions:
		// - points[2]
		// - edge points[0]-points[2]
		// - edge points[1]-points[2]
		// - inside the triangle
		public void Solve3() {
			b2Vec2 w1 = m_v1.w;
			b2Vec2 w2 = m_v2.w;
			b2Vec2 w3 = m_v3.w;

			// Edge12
			// [1      1     ][a1] = [1]
			// [w1.e12 w2.e12][a2] = [0]
			// a3 = 0
			b2Vec2 e12 = w2 - w1;
			float w1e12 = Utilities.b2Dot(w1, e12);
			float w2e12 = Utilities.b2Dot(w2, e12);
			float d12_1 = w2e12;
			float d12_2 = -w1e12;

			// Edge13
			// [1      1     ][a1] = [1]
			// [w1.e13 w3.e13][a3] = [0]
			// a2 = 0
			b2Vec2 e13 = w3 - w1;
			float w1e13 = Utilities.b2Dot(w1, e13);
			float w3e13 = Utilities.b2Dot(w3, e13);
			float d13_1 = w3e13;
			float d13_2 = -w1e13;

			// Edge23
			// [1      1     ][a2] = [1]
			// [w2.e23 w3.e23][a3] = [0]
			// a1 = 0
			b2Vec2 e23 = w3 - w2;
			float w2e23 = Utilities.b2Dot(w2, e23);
			float w3e23 = Utilities.b2Dot(w3, e23);
			float d23_1 = w3e23;
			float d23_2 = -w2e23;

			// Triangle123
			float n123 = Utilities.b2Cross(e12, e13);

			float d123_1 = n123 * Utilities.b2Cross(w2, w3);
			float d123_2 = n123 * Utilities.b2Cross(w3, w1);
			float d123_3 = n123 * Utilities.b2Cross(w1, w2);

			// w1 region
			if (d12_2 <= 0.0f && d13_2 <= 0.0f) {
				m_v1.a = 1.0f;
				m_count = 1;
				return;
			}

			// e12
			if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f) {
				float inv_d12 = 1.0f / (d12_1 + d12_2);
				m_v1.a = d12_1 * inv_d12;
				m_v2.a = d12_2 * inv_d12;
				m_count = 2;
				return;
			}

			// e13
			if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f) {
				float inv_d13 = 1.0f / (d13_1 + d13_2);
				m_v1.a = d13_1 * inv_d13;
				m_v3.a = d13_2 * inv_d13;
				m_count = 2;
				m_v2 = m_v3;
				return;
			}

			// w2 region
			if (d12_1 <= 0.0f && d23_2 <= 0.0f) {
				m_v2.a = 1.0f;
				m_count = 1;
				m_v1 = m_v2;
				return;
			}

			// w3 region
			if (d13_1 <= 0.0f && d23_1 <= 0.0f) {
				m_v3.a = 1.0f;
				m_count = 1;
				m_v1 = m_v3;
				return;
			}

			// e23
			if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f) {
				float inv_d23 = 1.0f / (d23_1 + d23_2);
				m_v2.a = d23_1 * inv_d23;
				m_v3.a = d23_2 * inv_d23;
				m_count = 2;
				m_v1 = m_v3;
				return;
			}

			// Must be in triangle123
			float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
			m_v1.a = d123_1 * inv_d123;
			m_v2.a = d123_2 * inv_d123;
			m_v3.a = d123_3 * inv_d123;
			m_count = 3;
		}


		public b2SimplexVertex m_v1, m_v2, m_v3;

		public b2SimplexVertex[] verticies {
			get {
				return new b2SimplexVertex[] { m_v1, m_v2, m_v3 };
			}
		}
		public int m_count;
	};
}
