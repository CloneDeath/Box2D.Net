
/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include <Box2D/Common/b2Math.h>

class b2Shape;



/// Used to warm start b2Distance.
/// Set count to zero on first call.
struct b2SimplexCache
{
	float metric;		///< length or area
	ushort count;
	byte indexA[3];	///< vertices on shape A
	byte indexB[3];	///< vertices on shape B
};

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
struct b2DistanceInput
{
	b2DistanceProxy proxyA;
	b2DistanceProxy proxyB;
	b2Transform transformA;
	b2Transform transformB;
	bool useRadii;
};

/// Output for b2Distance.
struct b2DistanceOutput
{
	b2Vec2 pointA;		///< closest point on shapeA
	b2Vec2 pointB;		///< closest point on shapeB
	float distance;
	int iterations;	///< number of GJK iterations used
};

/// Compute the closest points between two shapes. Supports any combination of:
/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
/// On the first call set b2SimplexCache.count to zero.
void b2Distance(b2DistanceOutput* output,
				b2SimplexCache* cache, 
				const b2DistanceInput* input);


//////////////////////////////////////////////////////////////////////////

inline int b2DistanceProxy::GetVertexCount() const
{
	return m_count;
}

inline const b2Vec2& b2DistanceProxy::GetVertex(int index) const
{
	Utilities.Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline int b2DistanceProxy::GetSupport(const b2Vec2& d) const
{
	int bestIndex = 0;
	float bestValue = Utilities.b2Dot(m_vertices[0], d);
	for (int i = 1; i < m_count; ++i)
	{
		float value = Utilities.b2Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return bestIndex;
}

inline const b2Vec2& b2DistanceProxy::GetSupportVertex(const b2Vec2& d) const
{
	int bestIndex = 0;
	float bestValue = Utilities.b2Dot(m_vertices[0], d);
	for (int i = 1; i < m_count; ++i)
	{
		float value = Utilities.b2Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return m_vertices[bestIndex];
}

#endif
