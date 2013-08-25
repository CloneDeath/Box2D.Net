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

#ifndef B2_MATH_H
#define B2_MATH_H

#include <Box2D/Common/Settings.h>

#include <cmath>
#include <cfloat>
#include <cstddef>
#include <limits>



/// This is a approximate yet fast inverse square-root.
inline float InvSqrt(float x)
{
	union
	{
		float x;
		int i;
	} convert;

	convert.x = x;
	float xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}






template <typename T>
inline T Math.Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline Vec2 Math.Abs(Vec2 a)
{
	return Vec2(Math.Abs(a.x), Math.Abs(a.y));
}

inline Mat22 Abs(const Mat22& A)
{
	return Mat22(Math.Abs(A.ex), Math.Abs(A.ey));
}




template<typename T> inline void Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint NextPowerOfTwo(uint x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

inline bool IsPowerOfTwo(uint x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}


#endif
