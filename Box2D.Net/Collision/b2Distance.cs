using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public static partial class Utilities {
		public static int _gjkCalls, _gjkIters, _gjkMaxIters;

		/// Compute the closest points between two shapes. Supports any combination of:
		/// CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
		/// On the first call set SimplexCache.count to zero.
		public static void Distance(out DistanceOutput output,
						SimplexCache cache,
						DistanceInput input)
		{
			++_gjkCalls;

			DistanceProxy proxyA = input.proxyA;
			DistanceProxy proxyB = input.proxyB;

			Transform transformA = input.transformA;
			Transform transformB = input.transformB;

			// Initialize the simplex.
			Simplex simplex = new Simplex();
			simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

			// Get simplex vertices as an array.
			SimplexVertex[] vertices = simplex.verticies;
			const int k_maxIters = 20;

			// These store the vertices of the last simplex so that we
			// can check for duplicates and prevent cycling.
			int[] saveA = new int[3];
			int[] saveB = new int[3];
			int saveCount = 0;

			float distanceSqr1 = Single.MaxValue;
			float distanceSqr2 = distanceSqr1;

			// Main iteration loop.
			int iter = 0;
			while (iter < k_maxIters)
			{
			    // Copy simplex so we can identify duplicates.
			    saveCount = simplex.m_count;
			    for (int i = 0; i < saveCount; ++i)
			    {
			        saveA[i] = vertices[i].indexA;
			        saveB[i] = vertices[i].indexB;
			    }

			    switch (simplex.m_count)
			    {
			    case 1:
			        break;

			    case 2:
			        simplex.Solve2();
			        break;

			    case 3:
			        simplex.Solve3();
			        break;

			    default:
			        Utilities.Assert(false);
					break;
			    }

			    // If we have 3 points, then the origin is in the corresponding triangle.
			    if (simplex.m_count == 3)
			    {
			        break;
			    }

			    // Compute closest point.
			    Vec2 p = simplex.GetClosestPoint();
			    distanceSqr2 = p.LengthSquared();

			    // Ensure progress
			    if (distanceSqr2 >= distanceSqr1)
			    {
			        //break;
			    }
			    distanceSqr1 = distanceSqr2;

			    // Get search direction.
			    Vec2 d = simplex.GetSearchDirection();

			    // Ensure the search direction is numerically fit.
			    if (d.LengthSquared() < Single.Epsilon * Single.Epsilon)
			    {
			        // The origin is probably contained by a line segment
			        // or triangle. Thus the shapes are overlapped.

			        // We can't return zero here even though there may be overlap.
			        // In case the simplex is a point, segment, or triangle it is difficult
			        // to determine if the origin is contained in the CSO or very close to it.
			        break;
			    }

			    // Compute a tentative new simplex vertex using support points.
			    SimplexVertex vertex = vertices[simplex.m_count];
			    vertex.indexA = proxyA.GetSupport(Utilities.MulT(transformA.q, -d));
			    vertex.wA = Utilities.Mul(transformA, proxyA.GetVertex(vertex.indexA));
			    Vec2 wBLocal;
			    vertex.indexB = proxyB.GetSupport(Utilities.MulT(transformB.q, d));
			    vertex.wB = Utilities.Mul(transformB, proxyB.GetVertex(vertex.indexB));
			    vertex.w = vertex.wB - vertex.wA;

			    // Iteration count is equated to the number of support point calls.
			    ++iter;
			    ++_gjkIters;

			    // Check for duplicate support points. This is the main termination criteria.
			    bool duplicate = false;
			    for (int i = 0; i < saveCount; ++i)
			    {
			        if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
			        {
			            duplicate = true;
			            break;
			        }
			    }

			    // If we found a duplicate support point we must exit to avoid cycling.
			    if (duplicate)
			    {
			        break;
			    }

			    // New vertex is ok and needed.
			    ++simplex.m_count;
			}

			_gjkMaxIters = Math.Max(_gjkMaxIters, iter);

			// Prepare output.
			simplex.GetWitnessPoints(out output.pointA, out output.pointB);
			output.distance = Utilities.Distance(output.pointA, output.pointB);
			output.iterations = iter;

			// Cache the simplex.
			simplex.WriteCache(cache);

			// Apply radii if requested.
			if (input.useRadii)
			{
			    float rA = proxyA.m_radius;
			    float rB = proxyB.m_radius;

			    if (output.distance > rA + rB && output.distance > Single.Epsilon)
			    {
			        // Shapes are still no overlapped.
			        // Move the witness points to the outer surface.
			        output.distance -= rA + rB;
			        Vec2 normal = output.pointB - output.pointA;
			        normal.Normalize();
			        output.pointA += rA * normal;
			        output.pointB -= rB * normal;
			    }
			    else
			    {
			        // Shapes are overlapped when radii are considered.
			        // Move the witness points to the middle.
			        Vec2 p = 0.5f * (output.pointA + output.pointB);
			        output.pointA = p;
			        output.pointB = p;
			        output.distance = 0.0f;
			    }
			}
		}


	}
}
