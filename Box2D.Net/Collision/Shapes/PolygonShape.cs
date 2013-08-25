using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A convex polygon. It is assumed that the interior of the polygon is to
	/// the left of each edge.
	/// Polygons have a maximum number of vertices equal to Settings._maxPolygonVertices.
	/// In most cases you should not need many vertices for a convex polygon.
	public class PolygonShape : Shape {
		public Vec2 m_centroid;
		public Vec2[] m_vertices = new Vec2[Settings._maxPolygonVertices];
		public Vec2[] m_normals = new Vec2[Settings._maxPolygonVertices];
		public int m_count;
		
		public PolygonShape(){
			m_type = ShapeType.Polygon;
			m_radius = Settings._polygonRadius;
			m_count = 0;
			m_centroid.SetZero();
		}

		/// Implement Shape.
		public override Shape Clone() {
			return (Shape)this.MemberwiseClone();
		}

		/// @see Shape::GetChildCount
		public override int GetChildCount() {
			return 1;
		}

		/// Create a convex hull from the given array of local points.
		/// The count must be in the range [3, Settings._maxPolygonVertices].
		/// @warning the points may be re-ordered, even if they form a convex polygon
		/// @warning collinear points are handled but not removed. Collinear points
		/// may lead to poor stacking behavior.
		public void Set(Vec2[] vertices, int count){
			Utilities.Assert(3 <= count && count <= Settings._maxPolygonVertices);
			if (count < 3)
			{
			    SetAsBox(1.0f, 1.0f);
			    return;
			}
			
			int n = Math.Min(count, Settings._maxPolygonVertices);

			// Copy vertices into local buffer
			Vec2[] ps = new Vec2[Settings._maxPolygonVertices];
			for (int i = 0; i < n; ++i)
			{
			    ps[i] = vertices[i];
			}

			// Create the convex hull using the Gift wrapping algorithm
			// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

			// Find the right most point on the hull
			int i0 = 0;
			float x0 = ps[0].x;
			for (int i = 1; i < count; ++i)
			{
			    float x = ps[i].x;
			    if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
			    {
			        i0 = i;
			        x0 = x;
			    }
			}

			int[] hull = new int[Settings._maxPolygonVertices];
			int m = 0;
			int ih = i0;

			for (;;)
			{
			    hull[m] = ih;

			    int ie = 0;
			    for (int j = 1; j < n; ++j)
			    {
			        if (ie == ih)
			        {
			            ie = j;
			            continue;
			        }

			        Vec2 r = ps[ie] - ps[hull[m]];
			        Vec2 v = ps[j] - ps[hull[m]];
			        float c = Utilities.Cross(r, v);
			        if (c < 0.0f)
			        {
			            ie = j;
			        }

			        // Collinearity check
			        if (c == 0.0f && v.LengthSquared() > r.LengthSquared())
			        {
			            ie = j;
			        }
			    }

			    ++m;
			    ih = ie;

			    if (ie == i0)
			    {
			        break;
			    }
			}
			
			m_count = m;

			// Copy vertices.
			for (int i = 0; i < m; ++i)
			{
			    m_vertices[i] = ps[hull[i]];
			}

			// Compute normals. Ensure the edges have non-zero length.
			for (int i = 0; i < m; ++i)
			{
			    int i1 = i;
			    int i2 = i + 1 < m ? i + 1 : 0;
			    Vec2 edge = m_vertices[i2] - m_vertices[i1];
			    Utilities.Assert(edge.LengthSquared() > Single.Epsilon * Single.Epsilon);
			    m_normals[i] = Utilities.Cross(edge, 1.0f);
			    m_normals[i].Normalize();
			}

			// Compute the polygon centroid.
			m_centroid = ComputeCentroid(m_vertices, m);
		}

		/// Build vertices to represent an axis-aligned box centered on the local origin.
		/// @param hx the half-width.
		/// @param hy the half-height.
		public void SetAsBox(float hx, float hy){
			m_count = 4;
			m_vertices[0].Set(-hx, -hy);
			m_vertices[1].Set( hx, -hy);
			m_vertices[2].Set( hx,  hy);
			m_vertices[3].Set(-hx,  hy);
			m_normals[0].Set(0.0f, -1.0f);
			m_normals[1].Set(1.0f, 0.0f);
			m_normals[2].Set(0.0f, 1.0f);
			m_normals[3].Set(-1.0f, 0.0f);
			m_centroid.SetZero();
		}


		/// Build vertices to represent an oriented box.
		/// @param hx the half-width.
		/// @param hy the half-height.
		/// @param center the center of the box in local coordinates.
		/// @param angle the rotation of the box in local coordinates.
		public void SetAsBox(float hx, float hy, Vec2 center, float angle){
			m_count = 4;
			m_vertices[0].Set(-hx, -hy);
			m_vertices[1].Set(hx, -hy);
			m_vertices[2].Set(hx, hy);
			m_vertices[3].Set(-hx, hy);
			m_normals[0].Set(0.0f, -1.0f);
			m_normals[1].Set(1.0f, 0.0f);
			m_normals[2].Set(0.0f, 1.0f);
			m_normals[3].Set(-1.0f, 0.0f);
			m_centroid = center;

			Transform xf = new Transform();
			xf.p = center;
			xf.q.Set(angle);

			// Transform vertices and normals.
			for (int i = 0; i < m_count; ++i) {
				m_vertices[i] = Utilities.Mul(xf, m_vertices[i]);
				m_normals[i] = Utilities.Mul(xf.q, m_normals[i]);
			}
		}
		
		private static Vec2 ComputeCentroid(Vec2[] vs, int count)
		{
		    Utilities.Assert(count >= 3);

		    Vec2 c = new Vec2(0.0f, 0.0f);
		    float area = 0.0f;

		    // pRef is the reference point for forming triangles.
		    // It's location doesn't change the result (except for rounding error).
		    Vec2 pRef = new Vec2(0.0f, 0.0f);
		#if ZERO
		    // This code would put the reference point inside the polygon.
		    for (int i = 0; i < count; ++i)
		    {
		        pRef += vs[i];
		    }
		    pRef *= 1.0f / count;
		#endif

		    const float inv3 = 1.0f / 3.0f;

		    for (int i = 0; i < count; ++i)
		    {
		        // Triangle vertices.
		        Vec2 p1 = pRef;
		        Vec2 p2 = vs[i];
		        Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

		        Vec2 e1 = p2 - p1;
		        Vec2 e2 = p3 - p1;

		        float D = Utilities.Cross(e1, e2);

		        float triangleArea = 0.5f * D;
		        area += triangleArea;

		        // Area weighted centroid
		        c += triangleArea * inv3 * (p1 + p2 + p3);
		    }

		    // Centroid
		    Utilities.Assert(area > Single.Epsilon);
		    c *= 1.0f / area;
		    return c;
		}

		/// @see Shape::TestPoint
		public override bool TestPoint(Transform transform, Vec2 p) {
			throw new NotImplementedException();
			//Vec2 pLocal = Utilities.MulT(xf.q, p - xf.p);

			//for (int i = 0; i < m_count; ++i)
			//{
			//    float dot = Utilities.Dot(m_normals[i], pLocal - m_vertices[i]);
			//    if (dot > 0.0f)
			//    {
			//        return false;
			//    }
			//}

			//return true;
		}

		/// Implement Shape.
		public override bool RayCast(out RayCastOutput output, RayCastInput input, Transform transform, int childIndex) {
			throw new NotImplementedException();

			//// Put the ray into the polygon's frame of reference.
			//Vec2 p1 = Utilities.MulT(xf.q, input.p1 - xf.p);
			//Vec2 p2 = Utilities.MulT(xf.q, input.p2 - xf.p);
			//Vec2 d = p2 - p1;

			//float lower = 0.0f, upper = input.maxFraction;

			//int index = -1;

			//for (int i = 0; i < m_count; ++i)
			//{
			//    // p = p1 + a * d
			//    // dot(normal, p - v) = 0
			//    // dot(normal, p1 - v) + a * dot(normal, d) = 0
			//    float numerator = Utilities.Dot(m_normals[i], m_vertices[i] - p1);
			//    float denominator = Utilities.Dot(m_normals[i], d);

			//    if (denominator == 0.0f)
			//    {	
			//        if (numerator < 0.0f)
			//        {
			//            return false;
			//        }
			//    }
			//    else
			//    {
			//        // Note: we want this predicate without division:
			//        // lower < numerator / denominator, where denominator < 0
			//        // Since denominator < 0, we have to flip the inequality:
			//        // lower < numerator / denominator <==> denominator * lower > numerator.
			//        if (denominator < 0.0f && numerator < lower * denominator)
			//        {
			//            // Increase lower.
			//            // The segment enters this half-space.
			//            lower = numerator / denominator;
			//            index = i;
			//        }
			//        else if (denominator > 0.0f && numerator < upper * denominator)
			//        {
			//            // Decrease upper.
			//            // The segment exits this half-space.
			//            upper = numerator / denominator;
			//        }
			//    }

			//    // The use of epsilon here causes the assert on lower to trip
			//    // in some cases. Apparently the use of epsilon was to make edge
			//    // shapes work, but now those are handled separately.
			//    //if (upper < lower - Single.Epsilon)
			//    if (upper < lower)
			//    {
			//        return false;
			//    }
			//}

			//Utilities.Assert(0.0f <= lower && lower <= input.maxFraction);

			//if (index >= 0)
			//{
			//    output.fraction = lower;
			//    output.normal = Utilities.Mul(xf.q, m_normals[index]);
			//    return true;
			//}

			//return false;
		}

		/// @see Shape::ComputeAABB
		public override void ComputeAABB(out AABB aabb, Transform xf, int childIndex) {
			Vec2 lower = Utilities.Mul(xf, m_vertices[0]);
			Vec2 upper = lower;

			for (int i = 1; i < m_count; ++i)
			{
			    Vec2 v = Utilities.Mul(xf, m_vertices[i]);
			    lower = Utilities.Min(lower, v);
			    upper = Utilities.Max(upper, v);
			}

			Vec2 r = new Vec2(m_radius, m_radius);
			aabb.lowerBound = lower - r;
			aabb.upperBound = upper + r;
		}

		/// @see Shape::ComputeMass
		public override void ComputeMass(out MassData massData, float density) {
			// Polygon mass, centroid, and inertia.
			// Let rho be the polygon density in mass per unit area.
			// Then:
			// mass = rho * int(dA)
			// centroid.x = (1/mass) * rho * int(x * dA)
			// centroid.y = (1/mass) * rho * int(y * dA)
			// I = rho * int((x*x + y*y) * dA)
			//
			// We can compute these integrals by summing all the integrals
			// for each triangle of the polygon. To evaluate the integral
			// for a single triangle, we make a change of variables to
			// the (u,v) coordinates of the triangle:
			// x = x0 + e1x * u + e2x * v
			// y = y0 + e1y * u + e2y * v
			// where 0 <= u && 0 <= v && u + v <= 1.
			//
			// We integrate u from [0,1-v] and then v from [0,1].
			// We also need to use the Jacobian of the transformation:
			// D = cross(e1, e2)
			//
			// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
			//
			// The rest of the derivation is handled by computer algebra.

			Utilities.Assert(m_count >= 3);

			Vec2 center = new Vec2(0.0f, 0.0f);
			float area = 0.0f;
			float I = 0.0f;

			// s is the reference point for forming triangles.
			// It's location doesn't change the result (except for rounding error).
			Vec2 s = new Vec2(0.0f, 0.0f);

			// This code would put the reference point inside the polygon.
			for (int i = 0; i < m_count; ++i)
			{
			    s += m_vertices[i];
			}
			s *= 1.0f / m_count;

			const float k_inv3 = 1.0f / 3.0f;

			for (int i = 0; i < m_count; ++i)
			{
			    // Triangle vertices.
			    Vec2 e1 = m_vertices[i] - s;
			    Vec2 e2 = i + 1 < m_count ? m_vertices[i+1] - s : m_vertices[0] - s;

			    float D = Utilities.Cross(e1, e2);

			    float triangleArea = 0.5f * D;
			    area += triangleArea;

			    // Area weighted centroid
			    center += triangleArea * k_inv3 * (e1 + e2);

			    float ex1 = e1.x, ey1 = e1.y;
			    float ex2 = e2.x, ey2 = e2.y;

			    float intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
			    float inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;

			    I += (0.25f * k_inv3 * D) * (intx2 + inty2);
			}

			// Total mass
			massData.mass = density * area;

			// Center of mass
			Utilities.Assert(area > Single.Epsilon);
			center *= 1.0f / area;
			massData.center = center + s;

			// Inertia tensor relative to the local origin (point s).
			massData.I = density * I;

			// Shift to center of mass then to original body origin.
			massData.I += massData.mass * (Utilities.Dot(massData.center, massData.center) - Utilities.Dot(center, center));
		}

		/// Get the vertex count.
		public int GetVertexCount() { return m_count; }

		/// Get a vertex by index.
		public Vec2 GetVertex(int index){
			throw new NotImplementedException();
			//Utilities.Assert(0 <= index && index < m_count);
			//return m_vertices[index];
		}

		/// Validate convexity. This is a very time consuming operation.
		/// @returns true if valid
		public bool Validate(){
			for (int i = 0; i < m_count; ++i) {
				int i1 = i;
				int i2 = i < m_count - 1 ? i1 + 1 : 0;
				Vec2 p = m_vertices[i1];
				Vec2 e = m_vertices[i2] - p;

				for (int j = 0; j < m_count; ++j) {
					if (j == i1 || j == i2) {
						continue;
					}

					Vec2 v = m_vertices[j] - p;
					float c = Utilities.Cross(e, v);
					if (c < 0.0f) {
						return false;
					}
				}
			}

			return true;
		}
	}
}
