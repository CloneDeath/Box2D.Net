/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2Distance.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Common/b2Timer.h>

#include <cstdio>
using namespace std;



//
struct b2SeparationFunction
{
	enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	};

	// TODO_ERIN might not need to return the separation

	float Initialize(const b2SimplexCache* cache,
		const b2DistanceProxy* proxyA, const b2Sweep& sweepA,
		const b2DistanceProxy* proxyB, const b2Sweep& sweepB,
		float t1)
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		int count = cache.count;
		Utilities.Assert(0 < count && count < 3);

		m_sweepA = sweepA;
		m_sweepB = sweepB;

		b2Transform xfA, xfB;
		m_sweepA.GetTransform(&xfA, t1);
		m_sweepB.GetTransform(&xfB, t1);

		if (count == 1)
		{
			m_type = e_points;
			b2Vec2 localPointA = m_proxyA.GetVertex(cache.indexA[0]);
			b2Vec2 localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			b2Vec2 pointA = Utilities.b2Mul(xfA, localPointA);
			b2Vec2 pointB = Utilities.b2Mul(xfB, localPointB);
			m_axis = pointB - pointA;
			float s = m_axis.Normalize();
			return s;
		}
		else if (cache.indexA[0] == cache.indexA[1])
		{
			// Two points on B and one on A.
			m_type = e_faceB;
			b2Vec2 localPointB1 = proxyB.GetVertex(cache.indexB[0]);
			b2Vec2 localPointB2 = proxyB.GetVertex(cache.indexB[1]);

			m_axis = Utilities.b2Cross(localPointB2 - localPointB1, 1.0f);
			m_axis.Normalize();
			b2Vec2 normal = Utilities.b2Mul(xfB.q, m_axis);

			m_localPoint = 0.5f * (localPointB1 + localPointB2);
			b2Vec2 pointB = Utilities.b2Mul(xfB, m_localPoint);

			b2Vec2 localPointA = proxyA.GetVertex(cache.indexA[0]);
			b2Vec2 pointA = Utilities.b2Mul(xfA, localPointA);

			float s = Utilities.b2Dot(pointA - pointB, normal);
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
			m_type = e_faceA;
			b2Vec2 localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
			b2Vec2 localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);
			
			m_axis = Utilities.b2Cross(localPointA2 - localPointA1, 1.0f);
			m_axis.Normalize();
			b2Vec2 normal = Utilities.b2Mul(xfA.q, m_axis);

			m_localPoint = 0.5f * (localPointA1 + localPointA2);
			b2Vec2 pointA = Utilities.b2Mul(xfA, m_localPoint);

			b2Vec2 localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			b2Vec2 pointB = Utilities.b2Mul(xfB, localPointB);

			float s = Utilities.b2Dot(pointB - pointA, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
				s = -s;
			}
			return s;
		}
	}

	//
	float FindMinSeparation(int* indexA, int* indexB, float t) const
	{
		b2Transform xfA, xfB;
		m_sweepA.GetTransform(&xfA, t);
		m_sweepB.GetTransform(&xfB, t);

		switch (m_type)
		{
		case e_points:
			{
				b2Vec2 axisA = Utilities.b2MulT(xfA.q,  m_axis);
				b2Vec2 axisB = Utilities.b2MulT(xfB.q, -m_axis);

				*indexA = m_proxyA.GetSupport(axisA);
				*indexB = m_proxyB.GetSupport(axisB);

				b2Vec2 localPointA = m_proxyA.GetVertex(*indexA);
				b2Vec2 localPointB = m_proxyB.GetVertex(*indexB);
				
				b2Vec2 pointA = Utilities.b2Mul(xfA, localPointA);
				b2Vec2 pointB = Utilities.b2Mul(xfB, localPointB);

				float separation = Utilities.b2Dot(pointB - pointA, m_axis);
				return separation;
			}

		case e_faceA:
			{
				b2Vec2 normal = Utilities.b2Mul(xfA.q, m_axis);
				b2Vec2 pointA = Utilities.b2Mul(xfA, m_localPoint);

				b2Vec2 axisB = Utilities.b2MulT(xfB.q, -normal);
				
				*indexA = -1;
				*indexB = m_proxyB.GetSupport(axisB);

				b2Vec2 localPointB = m_proxyB.GetVertex(*indexB);
				b2Vec2 pointB = Utilities.b2Mul(xfB, localPointB);

				float separation = Utilities.b2Dot(pointB - pointA, normal);
				return separation;
			}

		case e_faceB:
			{
				b2Vec2 normal = Utilities.b2Mul(xfB.q, m_axis);
				b2Vec2 pointB = Utilities.b2Mul(xfB, m_localPoint);

				b2Vec2 axisA = Utilities.b2MulT(xfA.q, -normal);

				*indexB = -1;
				*indexA = m_proxyA.GetSupport(axisA);

				b2Vec2 localPointA = m_proxyA.GetVertex(*indexA);
				b2Vec2 pointA = Utilities.b2Mul(xfA, localPointA);

				float separation = Utilities.b2Dot(pointA - pointB, normal);
				return separation;
			}

		default:
			Utilities.Assert(false);
			*indexA = -1;
			*indexB = -1;
			return 0.0f;
		}
	}

	//
	float Evaluate(int indexA, int indexB, float t) const
	{
		b2Transform xfA, xfB;
		m_sweepA.GetTransform(&xfA, t);
		m_sweepB.GetTransform(&xfB, t);

		switch (m_type)
		{
		case e_points:
			{
				b2Vec2 localPointA = m_proxyA.GetVertex(indexA);
				b2Vec2 localPointB = m_proxyB.GetVertex(indexB);

				b2Vec2 pointA = Utilities.b2Mul(xfA, localPointA);
				b2Vec2 pointB = Utilities.b2Mul(xfB, localPointB);
				float separation = Utilities.b2Dot(pointB - pointA, m_axis);

				return separation;
			}

		case e_faceA:
			{
				b2Vec2 normal = Utilities.b2Mul(xfA.q, m_axis);
				b2Vec2 pointA = Utilities.b2Mul(xfA, m_localPoint);

				b2Vec2 localPointB = m_proxyB.GetVertex(indexB);
				b2Vec2 pointB = Utilities.b2Mul(xfB, localPointB);

				float separation = Utilities.b2Dot(pointB - pointA, normal);
				return separation;
			}

		case e_faceB:
			{
				b2Vec2 normal = Utilities.b2Mul(xfB.q, m_axis);
				b2Vec2 pointB = Utilities.b2Mul(xfB, m_localPoint);

				b2Vec2 localPointA = m_proxyA.GetVertex(indexA);
				b2Vec2 pointA = Utilities.b2Mul(xfA, localPointA);

				float separation = Utilities.b2Dot(pointA - pointB, normal);
				return separation;
			}

		default:
			Utilities.Assert(false);
			return 0.0f;
		}
	}

	const b2DistanceProxy* m_proxyA;
	const b2DistanceProxy* m_proxyB;
	b2Sweep m_sweepA, m_sweepB;
	Type m_type;
	b2Vec2 m_localPoint;
	b2Vec2 m_axis;
};

