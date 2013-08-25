using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	//
	struct SeparationFunction
	{
		enum SeparationType
		{
			e_points,
			e_faceA,
			e_faceB
		};

		// TODO_ERIN might not need to return the separation

		float Initialize(SimplexCache cache,
			DistanceProxy proxyA, Sweep sweepA,
			DistanceProxy proxyB, Sweep sweepB,
			float t1)
		{
			m_proxyA = proxyA;
			m_proxyB = proxyB;
			int count = cache.count;
			Utilities.Assert(0 < count && count < 3);

			m_sweepA = sweepA;
			m_sweepB = sweepB;

			Transform xfA, xfB;
			m_sweepA.GetTransform(out xfA, t1);
			m_sweepB.GetTransform(out xfB, t1);

			if (count == 1)
			{
				m_type = SeparationType.e_points;
				Vec2 localPointA = m_proxyA.GetVertex(cache.indexA[0]);
				Vec2 localPointB = m_proxyB.GetVertex(cache.indexB[0]);
				Vec2 pointA = Utilities.Mul(xfA, localPointA);
				Vec2 pointB = Utilities.Mul(xfB, localPointB);
				m_axis = pointB - pointA;
				float s = m_axis.Normalize();
				return s;
			}
			else if (cache.indexA[0] == cache.indexA[1])
			{
				// Two points on B and one on A.
				m_type = SeparationType.e_faceB;
				Vec2 localPointB1 = proxyB.GetVertex(cache.indexB[0]);
				Vec2 localPointB2 = proxyB.GetVertex(cache.indexB[1]);

				m_axis = Utilities.Cross(localPointB2 - localPointB1, 1.0f);
				m_axis.Normalize();
				Vec2 normal = Utilities.Mul(xfB.q, m_axis);

				m_localPoint = 0.5f * (localPointB1 + localPointB2);
				Vec2 pointB = Utilities.Mul(xfB, m_localPoint);

				Vec2 localPointA = proxyA.GetVertex(cache.indexA[0]);
				Vec2 pointA = Utilities.Mul(xfA, localPointA);

				float s = Utilities.Dot(pointA - pointB, normal);
				if (s < 0.0f)
				{
					m_axis = -m_axis;
					s = -s;
				}
				return s;
			}
			else
			{
				// Two points on A and one or two points on B.
				m_type = SeparationType.e_faceA;
				Vec2 localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
				Vec2 localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);
			
				m_axis = Utilities.Cross(localPointA2 - localPointA1, 1.0f);
				m_axis.Normalize();
				Vec2 normal = Utilities.Mul(xfA.q, m_axis);

				m_localPoint = 0.5f * (localPointA1 + localPointA2);
				Vec2 pointA = Utilities.Mul(xfA, m_localPoint);

				Vec2 localPointB = m_proxyB.GetVertex(cache.indexB[0]);
				Vec2 pointB = Utilities.Mul(xfB, localPointB);

				float s = Utilities.Dot(pointB - pointA, normal);
				if (s < 0.0f)
				{
					m_axis = -m_axis;
					s = -s;
				}
				return s;
			}
		}

		//
		float FindMinSeparation(out int indexA, out int indexB, float t)
		{
			Transform xfA, xfB;
			m_sweepA.GetTransform(out xfA, t);
			m_sweepB.GetTransform(out xfB, t);

			switch (m_type)
			{
			case SeparationType.e_points:
				{
					Vec2 axisA = Utilities.MulT(xfA.q,  m_axis);
					Vec2 axisB = Utilities.MulT(xfB.q, -m_axis);

					indexA = m_proxyA.GetSupport(axisA);
					indexB = m_proxyB.GetSupport(axisB);

					Vec2 localPointA = m_proxyA.GetVertex(indexA);
					Vec2 localPointB = m_proxyB.GetVertex(indexB);
				
					Vec2 pointA = Utilities.Mul(xfA, localPointA);
					Vec2 pointB = Utilities.Mul(xfB, localPointB);

					float separation = Utilities.Dot(pointB - pointA, m_axis);
					return separation;
				}

			case SeparationType.e_faceA:
				{
					Vec2 normal = Utilities.Mul(xfA.q, m_axis);
					Vec2 pointA = Utilities.Mul(xfA, m_localPoint);

					Vec2 axisB = Utilities.MulT(xfB.q, -normal);
				
					indexA = -1;
					indexB = m_proxyB.GetSupport(axisB);

					Vec2 localPointB = m_proxyB.GetVertex(indexB);
					Vec2 pointB = Utilities.Mul(xfB, localPointB);

					float separation = Utilities.Dot(pointB - pointA, normal);
					return separation;
				}

			case SeparationType.e_faceB:
				{
					Vec2 normal = Utilities.Mul(xfB.q, m_axis);
					Vec2 pointB = Utilities.Mul(xfB, m_localPoint);

					Vec2 axisA = Utilities.MulT(xfA.q, -normal);

					indexB = -1;
					indexA = m_proxyA.GetSupport(axisA);

					Vec2 localPointA = m_proxyA.GetVertex(indexA);
					Vec2 pointA = Utilities.Mul(xfA, localPointA);

					float separation = Utilities.Dot(pointA - pointB, normal);
					return separation;
				}

			default:
				Utilities.Assert(false);
				indexA = -1;
				indexB = -1;
				return 0.0f;
			}
		}

		//
		float Evaluate(int indexA, int indexB, float t)
		{
			Transform xfA, xfB;
			m_sweepA.GetTransform(out xfA, t);
			m_sweepB.GetTransform(out xfB, t);

			switch (m_type)
			{
			case SeparationType.e_points:
				{
					Vec2 localPointA = m_proxyA.GetVertex(indexA);
					Vec2 localPointB = m_proxyB.GetVertex(indexB);

					Vec2 pointA = Utilities.Mul(xfA, localPointA);
					Vec2 pointB = Utilities.Mul(xfB, localPointB);
					float separation = Utilities.Dot(pointB - pointA, m_axis);

					return separation;
				}

			case SeparationType.e_faceA:
				{
					Vec2 normal = Utilities.Mul(xfA.q, m_axis);
					Vec2 pointA = Utilities.Mul(xfA, m_localPoint);

					Vec2 localPointB = m_proxyB.GetVertex(indexB);
					Vec2 pointB = Utilities.Mul(xfB, localPointB);

					float separation = Utilities.Dot(pointB - pointA, normal);
					return separation;
				}

			case SeparationType.e_faceB:
				{
					Vec2 normal = Utilities.Mul(xfB.q, m_axis);
					Vec2 pointB = Utilities.Mul(xfB, m_localPoint);

					Vec2 localPointA = m_proxyA.GetVertex(indexA);
					Vec2 pointA = Utilities.Mul(xfA, localPointA);

					float separation = Utilities.Dot(pointA - pointB, normal);
					return separation;
				}

			default:
				Utilities.Assert(false);
				return 0.0f;
			}
		}

		DistanceProxy m_proxyA;
		DistanceProxy m_proxyB;
		Sweep m_sweepA, m_sweepB;
		SeparationType m_type;
		Vec2 m_localPoint;
		Vec2 m_axis;
	}
}
