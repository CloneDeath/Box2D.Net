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

// From Real-time Collision Detection, p179.
bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input)
{
	float tmin = -Single.MaxValue;
	float tmax = Single.MaxValue;

	b2Vec2 p = input.p1;
	b2Vec2 d = input.p2 - input.p1;
	b2Vec2 absD = Math.Abs(d);

	b2Vec2 normal;

	for (int i = 0; i < 2; ++i)
	{
		if (absD(i) < Single.Epsilon)
		{
			// Parallel.
			if (p(i) < lowerBound(i) || upperBound(i) < p(i))
			{
				return false;
			}
		}
		else
		{
			float inv_d = 1.0f / d(i);
			float t1 = (lowerBound(i) - p(i)) * inv_d;
			float t2 = (upperBound(i) - p(i)) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if (t1 > t2)
			{
				b2Swap(t1, t2);
				s = 1.0f;
			}

			// Push the min up
			if (t1 > tmin)
			{
				normal.SetZero();
				normal(i) = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = Math.Min(tmax, t2);

			if (tmin > tmax)
			{
				return false;
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if (tmin < 0.0f || input.maxFraction < tmin)
	{
		return false;
	}

	// Intersection.
	output.fraction = tmin;
	output.normal = normal;
	return true;
}
