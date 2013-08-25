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

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>


// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
void CollideEdgeAndCircle(Manifold manifold,
							const EdgeShape* edgeA, const Transform& xfA,
							const CircleShape* circleB, const Transform& xfB)
{
	manifold.pointCount = 0;
	
	// Compute circle in frame of edge
	Vec2 Q = Utilities.MulT(xfA, Utilities.Mul(xfB, circleB.m_p));
	
	Vec2 A = edgeA.m_vertex1, B = edgeA.m_vertex2;
	Vec2 e = B - A;
	
	// Barycentric coordinates
	float u = Utilities.Dot(e, B - Q);
	float v = Utilities.Dot(e, Q - A);
	
	float radius = edgeA.m_radius + circleB.m_radius;
	
	ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0f)
	{
		Vec2 P = A;
		Vec2 d = Q - P;
		float dd = Utilities.Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}
		
		// Is there an edge connected to A?
		if (edgeA.m_hasVertex0)
		{
			Vec2 A1 = edgeA.m_vertex0;
			Vec2 B1 = A;
			Vec2 e1 = B1 - A1;
			float u1 = Utilities.Dot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0f)
			{
				return;
			}
		}
		
		cf.indexA = 0;
		cf.typeA = ContactFeature::e_vertex;
		manifold.pointCount = 1;
		manifold.type = Manifold::e_circles;
		manifold.localNormal.SetZero();
		manifold.localPoint = P;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_p;
		return;
	}
	
	// Region B
	if (u <= 0.0f)
	{
		Vec2 P = B;
		Vec2 d = Q - P;
		float dd = Utilities.Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}
		
		// Is there an edge connected to B?
		if (edgeA.m_hasVertex3)
		{
			Vec2 B2 = edgeA.m_vertex3;
			Vec2 A2 = B;
			Vec2 e2 = B2 - A2;
			float v2 = Utilities.Dot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0f)
			{
				return;
			}
		}
		
		cf.indexA = 1;
		cf.typeA = ContactFeature::e_vertex;
		manifold.pointCount = 1;
		manifold.type = Manifold::e_circles;
		manifold.localNormal.SetZero();
		manifold.localPoint = P;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_p;
		return;
	}
	
	// Region AB
	float den = Utilities.Dot(e, e);
	Utilities.Assert(den > 0.0f);
	Vec2 P = (1.0f / den) * (u * A + v * B);
	Vec2 d = Q - P;
	float dd = Utilities.Dot(d, d);
	if (dd > radius * radius)
	{
		return;
	}
	
	Vec2 n(-e.y, e.x);
	if (Utilities.Dot(n, Q - A) < 0.0f)
	{
		n.Set(-n.x, -n.y);
	}
	n.Normalize();
	
	cf.indexA = 0;
	cf.typeA = ContactFeature::e_face;
	manifold.pointCount = 1;
	manifold.type = Manifold::e_faceA;
	manifold.localNormal = n;
	manifold.localPoint = A;
	manifold.points[0].id.key = 0;
	manifold.points[0].id.cf = cf;
	manifold.points[0].localPoint = circleB.m_p;
}
