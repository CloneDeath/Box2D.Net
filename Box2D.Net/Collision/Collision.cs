using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Box2D {
	public class Collision {
		public const byte _nullFeature = byte.MaxValue;
		
		/// Compute the collision manifold between two circles.
		public static void CollideCircles(out Manifold manifold,
							  CircleShape circleA, Transform xfA,
							  CircleShape circleB, Transform xfB) {
			throw new NotImplementedException();
			//manifold.pointCount = 0;

			//Vec2 pA = Utilities.Mul(xfA, circleA.m_p);
			//Vec2 pB = Utilities.Mul(xfB, circleB.m_p);

			//Vec2 d = pB - pA;
			//float distSqr = Utilities.Dot(d, d);
			//float rA = circleA.m_radius, rB = circleB.m_radius;
			//float radius = rA + rB;
			//if (distSqr > radius * radius)
			//{
			//    return;
			//}

			//manifold.type = Manifold::e_circles;
			//manifold.localPoint = circleA.m_p;
			//manifold.localNormal.SetZero();
			//manifold.pointCount = 1;

			//manifold.points[0].localPoint = circleB.m_p;
			//manifold.points[0].id.key = 0;
		}

		/// Compute the collision manifold between a polygon and a circle.
		public static void CollidePolygonAndCircle(out Manifold manifold,
									   PolygonShape polygonA, Transform xfA,
									   CircleShape circleB, Transform xfB) {
			manifold = new Manifold();
			manifold.points.Clear();

			// Compute circle position in the frame of the polygon.
			Vec2 c = Utilities.Mul(xfB, circleB.m_position);
			Vec2 cLocal = Utilities.MulT(xfA, c);

			// Find the min separating edge.
			int normalIndex = 0;
			float separation = -Single.MaxValue;
			float radius = polygonA.m_radius + circleB.m_radius;
			int vertexCount = polygonA.m_count;
			List<Vec2> vertices = new List<Vec2>(polygonA.m_vertices);
			List<Vec2> normals = new List<Vec2>(polygonA.m_normals);

			for (int i = 0; i < vertexCount; ++i)
			{
			    float s = Utilities.Dot(normals[i], cLocal - vertices[i]);

			    if (s > radius)
			    {
			        // Early out.
			        return;
			    }

			    if (s > separation)
			    {
			        separation = s;
			        normalIndex = i;
			    }
			}

			// Vertices that subtend the incident face.
			int vertIndex1 = normalIndex;
			int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
			Vec2 v1 = vertices[vertIndex1];
			Vec2 v2 = vertices[vertIndex2];

			// If the center is inside the polygon ...
			if (separation < Single.Epsilon)
			{
				manifold.points.Clear();
				manifold.points.Add(new ManifoldPoint());
			    manifold.type = Manifold.ManifoldType.e_faceA;
			    manifold.localNormal = normals[normalIndex];
			    manifold.localPoint = 0.5f * (v1 + v2);
			    manifold.points[0].localPoint = circleB.m_position;
			    manifold.points[0].id.key = 0;
			    return;
			}

			// Compute barycentric coordinates
			float u1 = Utilities.Dot(cLocal - v1, v2 - v1);
			float u2 = Utilities.Dot(cLocal - v2, v1 - v2);
			if (u1 <= 0.0f)
			{
			    if (Utilities.DistanceSquared(cLocal, v1) > radius * radius)
			    {
			        return;
			    }

				manifold.points.Clear();
				manifold.points.Add(new ManifoldPoint());
			    manifold.type = Manifold.ManifoldType.e_faceA;
			    manifold.localNormal = cLocal - v1;
			    manifold.localNormal.Normalize();
			    manifold.localPoint = v1;
			    manifold.points[0].localPoint = circleB.m_position;
			    manifold.points[0].id.key = 0;
			}
			else if (u2 <= 0.0f)
			{
				if (Utilities.DistanceSquared(cLocal, v2) > radius * radius)
			    {
			        return;
			    }

				manifold.points = new List<ManifoldPoint>();
				manifold.points.Add(new ManifoldPoint());
			    manifold.type = Manifold.ManifoldType.e_faceA;
			    manifold.localNormal = cLocal - v2;
			    manifold.localNormal.Normalize();
			    manifold.localPoint = v2;
			    manifold.points[0].localPoint = circleB.m_position;
			    manifold.points[0].id.key = 0;
			}
			else
			{
			    Vec2 faceCenter = 0.5f * (v1 + v2);
			    float separation2 = Utilities.Dot(cLocal - faceCenter, normals[vertIndex1]);
			    if (separation2 > radius)
			    {
			        return;
			    }

				manifold.points = new List<ManifoldPoint>();
				manifold.points.Add(new ManifoldPoint());
			    manifold.type = Manifold.ManifoldType.e_faceA;
			    manifold.localNormal = normals[vertIndex1];
			    manifold.localPoint = faceCenter;
			    manifold.points[0].localPoint = circleB.m_position;
			    manifold.points[0].id.key = 0;
			}
		}

		// Find the separation between poly1 and poly2 for a give edge normal on poly1.
		static float EdgeSeparation(PolygonShape poly1, Transform xf1, int edge1,
									  PolygonShape poly2, Transform xf2)
		{
			Vec2[] vertices1 = poly1.m_vertices;
			Vec2[] normals1 = poly1.m_normals;

			int count2 = poly2.m_count;
			Vec2[] vertices2 = poly2.m_vertices;

			Utilities.Assert(0 <= edge1 && edge1 < poly1.m_count);

			// Convert normal from poly1's frame into poly2's frame.
			Vec2 normal1World = Utilities.Mul(xf1.q, normals1[edge1]);
			Vec2 normal1 = Utilities.MulT(xf2.q, normal1World);

			// Find support vertex on poly2 for -normal.
			int index = 0;
			float minDot = Single.MaxValue;

			for (int i = 0; i < count2; ++i)
			{
				float dot = Utilities.Dot(vertices2[i], normal1);
				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

			Vec2 v1 = Utilities.Mul(xf1, vertices1[edge1]);
			Vec2 v2 = Utilities.Mul(xf2, vertices2[index]);
			float separation = Utilities.Dot(v2 - v1, normal1World);
			return separation;
		}


		// Find the max separation between poly1 and poly2 using edge normals from poly1.
		static float FindMaxSeparation(out int edgeIndex,
										 PolygonShape poly1, Transform xf1,
										 PolygonShape poly2, Transform xf2)
		{
			int count1 = poly1.m_count;
			Vec2[] normals1 = poly1.m_normals;

			// Vector pointing from the centroid of poly1 to the centroid of poly2.
			Vec2 d = Utilities.Mul(xf2, poly2.m_centroid) - Utilities.Mul(xf1, poly1.m_centroid);
			Vec2 dLocal1 = Utilities.MulT(xf1.q, d);

			// Find edge normal on poly1 that has the largest projection onto d.
			int edge = 0;
			float maxDot = -Single.MaxValue;
			for (int i = 0; i < count1; ++i)
			{
				float dot = Utilities.Dot(normals1[i], dLocal1);
				if (dot > maxDot)
				{
					maxDot = dot;
					edge = i;
				}
			}

			// Get the separation for the edge normal.
			float s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);

			// Check the separation for the previous edge normal.
			int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
			float sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

			// Check the separation for the next edge normal.
			int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
			float sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

			// Find the best edge and the search direction.
			int bestEdge;
			float bestSeparation;
			int increment;
			if (sPrev > s && sPrev > sNext)
			{
				increment = -1;
				bestEdge = prevEdge;
				bestSeparation = sPrev;
			}
			else if (sNext > s)
			{
				increment = 1;
				bestEdge = nextEdge;
				bestSeparation = sNext;
			}
			else
			{
				edgeIndex = edge;
				return s;
			}

			// Perform a local search for the best edge normal.
			for ( ; ; )
			{
				if (increment == -1)
					edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
				else
					edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

				s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);

				if (s > bestSeparation)
				{
					bestEdge = edge;
					bestSeparation = s;
				}
				else
				{
					break;
				}
			}

			edgeIndex = bestEdge;
			return bestSeparation;
		}

		static void FindIncidentEdge(ClipVertex[/*2*/] c,
							 PolygonShape poly1, Transform xf1, int edge1,
							 PolygonShape poly2, Transform xf2)
		{
			Vec2[] normals1 = poly1.m_normals;

			int count2 = poly2.m_count;
			Vec2[] vertices2 = poly2.m_vertices;
			Vec2[] normals2 = poly2.m_normals;

			Utilities.Assert(0 <= edge1 && edge1 < poly1.m_count);

			// Get the normal of the reference edge in poly2's frame.
			Vec2 normal1 = Utilities.MulT(xf2.q, Utilities.Mul(xf1.q, normals1[edge1]));

			// Find the incident edge on poly2.
			int index = 0;
			float minDot = Single.MaxValue;
			for (int i = 0; i < count2; ++i)
			{
				float dot = Utilities.Dot(normal1, normals2[i]);
				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

			// Build the clip vertices for the incident edge.
			int i1 = index;
			int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

			c[0].v = Utilities.Mul(xf2, vertices2[i1]);
			c[0].id.cf.indexA = (byte)edge1;
			c[0].id.cf.indexB = (byte)i1;
			c[0].id.cf.typeA = ContactFeature.FeatureType.e_face;
			c[0].id.cf.typeB = ContactFeature.FeatureType.e_vertex;

			c[1].v = Utilities.Mul(xf2, vertices2[i2]);
			c[1].id.cf.indexA = (byte)edge1;
			c[1].id.cf.indexB = (byte)i2;
			c[1].id.cf.typeA = ContactFeature.FeatureType.e_face;
			c[1].id.cf.typeB = ContactFeature.FeatureType.e_vertex;
		}

		// Find edge normal of max separation on A - return if separating axis is found
		// Find edge normal of max separation on B - return if separation axis is found
		// Choose reference edge as min(minA, minB)
		// Find incident edge
		// Clip

		// The normal points from 1 to 2
		/// Compute the collision manifold between two polygons.
		public static void CollidePolygons(out Manifold manifold,
							   PolygonShape polyA, Transform xfA,
							   PolygonShape polyB, Transform xfB) {
			manifold = new Manifold();
			float totalRadius = polyA.m_radius + polyB.m_radius;

			int edgeA = 0;
			float separationA = FindMaxSeparation(out edgeA, polyA, xfA, polyB, xfB);
			if (separationA > totalRadius)
			    return;

			int edgeB = 0;
			float separationB = FindMaxSeparation(out edgeB, polyB, xfB, polyA, xfA);
			if (separationB > totalRadius)
			    return;

			PolygonShape poly1;	// reference polygon
			PolygonShape poly2;	// incident polygon
			Transform xf1, xf2;
			int edge1;		// reference edge
			bool flip;
			const float k_relativeTol = 0.98f;
			const float k_absoluteTol = 0.001f;

			if (separationB > k_relativeTol * separationA + k_absoluteTol)
			{
			    poly1 = polyB;
			    poly2 = polyA;
			    xf1 = xfB;
			    xf2 = xfA;
			    edge1 = edgeB;
			    manifold.type = Manifold.ManifoldType.e_faceB;
			    flip = true;
			}
			else
			{
			    poly1 = polyA;
			    poly2 = polyB;
			    xf1 = xfA;
			    xf2 = xfB;
			    edge1 = edgeA;
			    manifold.type = Manifold.ManifoldType.e_faceA;
			    flip = false;
			}

			ClipVertex[] incidentEdge = new ClipVertex[2];
			FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

			int count1 = poly1.m_count;
			Vec2[] vertices1 = poly1.m_vertices;

			int iv1 = edge1;
			int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

			Vec2 v11 = vertices1[iv1];
			Vec2 v12 = vertices1[iv2];

			Vec2 localTangent = v12 - v11;
			localTangent.Normalize();
	
			Vec2 localNormal = Utilities.Cross(localTangent, 1.0f);
			Vec2 planePoint = 0.5f * (v11 + v12);

			Vec2 tangent = Utilities.Mul(xf1.q, localTangent);
			Vec2 normal = Utilities.Cross(tangent, 1.0f);
	
			v11 = Utilities.Mul(xf1, v11);
			v12 = Utilities.Mul(xf1, v12);

			// Face offset.
			float frontOffset = Utilities.Dot(normal, v11);

			// Side offsets, extended by polytope skin thickness.
			float sideOffset1 = -Utilities.Dot(tangent, v11) + totalRadius;
			float sideOffset2 = Utilities.Dot(tangent, v12) + totalRadius;

			// Clip incident edge against extruded edge1 side edges.
			ClipVertex[] clipPoints1 = new ClipVertex[2];
			ClipVertex[] clipPoints2 = new ClipVertex[2];
			int np;

			// Clip to box side 1
			np = ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

			if (np < 2)
			    return;

			// Clip to negative box side 1
			np = ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

			if (np < 2)
			{
			    return;
			}

			// Now clipPoints2 contains the clipped points.
			manifold.localNormal = localNormal;
			manifold.localPoint = planePoint;

			manifold.points.Clear();
			for (int i = 0; i < Settings._maxManifoldPoints; ++i)
			{
			    float separation = Utilities.Dot(normal, clipPoints2[i].v) - frontOffset;

			    if (separation <= totalRadius)
			    {
					ManifoldPoint cp = new ManifoldPoint();
			        cp.localPoint = Utilities.MulT(xf2, clipPoints2[i].v);
			        cp.id = clipPoints2[i].id;
			        if (flip)
			        {
			            // Swap features
			            ContactFeature cf = cp.id.cf;
			            cp.id.cf.indexA = cf.indexB;
			            cp.id.cf.indexB = cf.indexA;
			            cp.id.cf.typeA = cf.typeB;
			            cp.id.cf.typeB = cf.typeA;
			        }
					manifold.points.Add(cp);
			    }
			}
		}


		/// Compute the collision manifold between an edge and a circle.
		public static void CollideEdgeAndCircle(out Manifold manifold,
									   EdgeShape edgeA, Transform xfA,
									   CircleShape circleB, Transform xfB){
			manifold = new Manifold();
	
			// Compute circle in frame of edge
			Vec2 Q = Utilities.MulT(xfA, Utilities.Mul(xfB, circleB.m_position));
	
			Vec2 A = edgeA.m_vertex1, B = edgeA.m_vertex2;
			Vec2 e = B - A;
	
			// Barycentric coordinates
			float u = Utilities.Dot(e, B - Q);
			float v = Utilities.Dot(e, Q - A);
	
			float radius = edgeA.m_radius + circleB.m_radius;
	
			ContactFeature cf;
			cf.indexB = 0;
			cf.typeB = ContactFeature.FeatureType.e_vertex;
	
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
				cf.typeA = ContactFeature.FeatureType.e_vertex;
				manifold.points.Clear();
				manifold.points.Add(new ManifoldPoint());
				manifold.type = Manifold.ManifoldType.e_circles;
				manifold.localNormal.SetZero();
				manifold.localPoint = P;
				manifold.points[0].id.key = 0;
				manifold.points[0].id.cf = cf;
				manifold.points[0].localPoint = circleB.m_position;
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
				cf.typeA = ContactFeature.FeatureType.e_vertex;
				manifold.points.Clear();
				manifold.points.Add(new ManifoldPoint());
				manifold.type = Manifold.ManifoldType.e_circles;
				manifold.localNormal.SetZero();
				manifold.localPoint = P;
				manifold.points[0].id.key = 0;
				manifold.points[0].id.cf = cf;
				manifold.points[0].localPoint = circleB.m_position;
				return;
			}
	
			// Region AB
			float den = Utilities.Dot(e, e);
			Utilities.Assert(den > 0.0f);
			Vec2 Pb = (1.0f / den) * (u * A + v * B);
			Vec2 db = Q - Pb;
			float ddb = Utilities.Dot(db, db);
			if (ddb > radius * radius)
			{
				return;
			}
	
			Vec2 n = new Vec2(-e.Y, e.X);
			if (Utilities.Dot(n, Q - A) < 0.0f)
			{
				n.Set(-n.X, -n.Y);
			}
			n.Normalize();
	
			cf.indexA = 0;
			cf.typeA = ContactFeature.FeatureType.e_face;
			manifold.points.Clear();
			manifold.points.Add(new ManifoldPoint());
			manifold.type = Manifold.ManifoldType.e_faceA;
			manifold.localNormal = n;
			manifold.localPoint = A;
			manifold.points[0].id.key = 0;
			manifold.points[0].id.cf = cf;
			manifold.points[0].localPoint = circleB.m_position;
		}

		/// Compute the collision manifold between an edge and a circle.
		public static void CollideEdgeAndPolygon(out Manifold manifold,
									   EdgeShape edgeA, Transform xfA,
									   PolygonShape polygonB, Transform xfB) {
			EPCollider collider = new EPCollider();
			collider.Collide(out manifold, edgeA, xfA, polygonB, xfB);
		}

		/// Clipping for contact manifolds.
		// Sutherland-Hodgman clipping.
		public static int ClipSegmentToLine(ClipVertex[/*2*/] vOut, ClipVertex[/*2*/] vIn,
									Vec2 normal, float offset, int vertexIndexA){
			// Start with no output points
			int numOut = 0;

			// Calculate the distance of end points to the line
			float distance0 = Utilities.Dot(normal, vIn[0].v) - offset;
			float distance1 = Utilities.Dot(normal, vIn[1].v) - offset;

			// If the points are behind the plane
			if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
			if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

			// If the points are on different sides of the plane
			if (distance0 * distance1 < 0.0f)
			{
				// Find intersection point of edge and plane
				float interp = distance0 / (distance0 - distance1);
				vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

				// VertexA is hitting edgeB.
				vOut[numOut].id.cf.indexA = (byte)vertexIndexA;
				vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
				vOut[numOut].id.cf.typeA = ContactFeature.FeatureType.e_vertex;
				vOut[numOut].id.cf.typeB = ContactFeature.FeatureType.e_face;
				++numOut;
			}

			return numOut;
		}

		/// Determine if two generic shapes overlap.
		public static bool TestOverlap(Shape shapeA, int indexA,
							Shape shapeB, int indexB,
							Transform xfA, Transform xfB){

			DistanceInput input = new DistanceInput();
			input.proxyA.Set(shapeA, indexA);
			input.proxyB.Set(shapeB, indexB);
			input.transformA = xfA;
			input.transformB = xfB;
			input.useRadii = true;

			SimplexCache cache = new SimplexCache();
			cache.count = 0;

			DistanceOutput output;

			Utilities.Distance(out output, cache, input);

			return output.distance < 10.0f * Single.Epsilon;
		}
		
		/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
		/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
		public static void GetPointStates(PointState[/*Settings._maxManifoldPoints*/] state1, PointState[/*Settings._maxManifoldPoints*/] state2,
							  Manifold manifold1, Manifold manifold2) {
			throw new NotImplementedException();
			//for (int i = 0; i < Settings._maxManifoldPoints; ++i)
			//{
			//    state1[i] = _nullState;
			//    state2[i] = _nullState;
			//}

			//// Detect persists and removes.
			//for (int i = 0; i < manifold1.pointCount; ++i)
			//{
			//    ContactID id = manifold1.points[i].id;

			//    state1[i] = _removeState;

			//    for (int j = 0; j < manifold2.pointCount; ++j)
			//    {
			//        if (manifold2.points[j].id.key == id.key)
			//        {
			//            state1[i] = _persistState;
			//            break;
			//        }
			//    }
			//}

			//// Detect persists and adds.
			//for (int i = 0; i < manifold2.pointCount; ++i)
			//{
			//    ContactID id = manifold2.points[i].id;

			//    state2[i] = _addState;

			//    for (int j = 0; j < manifold1.pointCount; ++j)
			//    {
			//        if (manifold1.points[j].id.key == id.key)
			//        {
			//            state2[i] = _persistState;
			//            break;
			//        }
			//    }
			//}
		}

		public static bool TestOverlap(AABB a, AABB b) {
			Vec2 d1, d2;
			d1 = b.lowerBound - a.upperBound;
			d2 = a.lowerBound - b.upperBound;

			if (d1.X > 0.0f || d1.Y > 0.0f)
				return false;

			if (d2.X > 0.0f || d2.Y > 0.0f)
				return false;

			return true;					
		}
		//{
		//    return (a.lowerBound.X <= b.upperBound.X) && (a.lowerBound.Y <= b.upperBound.Y) &&
		//           (a.upperBound.X >= b.lowerBound.X) && (a.upperBound.Y >= b.lowerBound.Y);
		//}
	}

	/// The features that intersect to form the contact point
	/// This must be 4 bytes or less.
	public struct ContactFeature
	{
		public enum FeatureType : byte
		{
			e_vertex = 0,
			e_face = 1
		};

		public byte indexA;		///< Feature index on shapeA
		public byte indexB;		///< Feature index on shapeB
		public FeatureType typeA;		///< The feature type on shapeA
		public FeatureType typeB;		///< The feature type on shapeB
	}

	/// Contact ids to facilitate warm starting.
	public struct ContactID
	{
		public ContactFeature cf;
		public uint key {
			get {
				return (uint)((cf.indexA << 24) & (cf.indexB << 16) & ((byte)cf.typeA << 8) & ((byte)cf.typeB));
			}

			set {
				cf.indexA = (byte)((value >> 24) & 0x00FF);
				cf.indexB = (byte)((value >> 16) & 0x00FF);
				cf.typeA = (ContactFeature.FeatureType)((value >> 8) & 0x00FF);
				cf.typeB = (ContactFeature.FeatureType)((value) & 0x00FF);
			}
		}
	}

	/// A manifold point is a contact point belonging to a contact
	/// manifold. It holds details related to the geometry and dynamics
	/// of the contact points.
	/// The local point usage depends on the manifold type:
	/// -e_circles: the local center of circleB
	/// -e_faceA: the local center of cirlceB or the clip point of polygonB
	/// -e_faceB: the clip point of polygonA
	/// This structure is stored across time steps, so we keep it small.
	/// Note: the impulses are used for internal caching and may not
	/// provide reliable contact forces, especially for high speed collisions.
	public class ManifoldPoint //was struct
	{
		public Vec2 localPoint;		///< usage depends on manifold type
		public float normalImpulse;	///< the non-penetration impulse
		public float tangentImpulse;	///< the friction impulse
		public ContactID id;			///< uniquely identifies a contact point between two shapes
	}

	/// A manifold for two touching convex shapes.
	/// Box2D supports multiple types of contact:
	/// - clip point versus plane with radius
	/// - point versus point with radius (circles)
	/// The local point usage depends on the manifold type:
	/// -e_circles: the local center of circleA
	/// -e_faceA: the center of faceA
	/// -e_faceB: the center of faceB
	/// Similarly the local normal usage:
	/// -e_circles: not used
	/// -e_faceA: the normal on polygonA
	/// -e_faceB: the normal on polygonB
	/// We store contacts in this way so that position correction can
	/// account for movement, which is critical for continuous physics.
	/// All contact scenarios must be expressed in one of these types.
	/// This structure is stored across time steps, so we keep it small.
	public class Manifold
	{
		public Manifold() {
			points = new List<ManifoldPoint>();
			localNormal = new Vec2();
			localPoint = new Vec2();
		}

		public Manifold(ManifoldType type) {
			points = new List<ManifoldPoint>();
			localNormal = new Vec2(0, 0);
			localPoint = new Vec2(0, 0);
			this.type = type;
		}

		public enum ManifoldType
		{
			e_circles,
			e_faceA,
			e_faceB
		};

		public List<ManifoldPoint> points;	///< the points of contact
		public Vec2 localNormal;								///< not use for Type::e_points
		public Vec2 localPoint;								///< usage depends on manifold type
		public ManifoldType type;
		//public int pointCount;								///< the number of manifold points
	}

	/// This is used to compute the current state of a contact manifold.
	public class WorldManifold //was struct
	{
		public WorldManifold() {
			points = new List<Vec2>();
		}
		public WorldManifold(Vec2 normal) {
			points = new List<Vec2>();
			this.normal = normal;
		}

		/// Evaluate the manifold with supplied transforms. This assumes
		/// modest motion from the original state. This does not change the
		/// point count, impulses, etc. The radii must come from the shapes
		/// that generated the manifold.
		public void Initialize(Manifold manifold,
								Transform xfA, float radiusA,
								Transform xfB, float radiusB){
			if (manifold.points.Count() == 0)
			{
				return;
			}

			switch (manifold.type)
			{
			case Manifold.ManifoldType.e_circles:
				{
					normal.Set(1.0f, 0.0f);
					Vec2 pointA = Utilities.Mul(xfA, manifold.localPoint);
					Vec2 pointB = Utilities.Mul(xfB, manifold.points[0].localPoint);
					if (Utilities.DistanceSquared(pointA, pointB) > Single.Epsilon * Single.Epsilon)
					{
						normal = pointB - pointA;
						normal.Normalize();
					}

					Vec2 cA = pointA + radiusA * normal;
					Vec2 cB = pointB - radiusB * normal;
					points[0] = 0.5f * (cA + cB);
				}
				break;

			case Manifold.ManifoldType.e_faceA:
				{
					normal = Utilities.Mul(xfA.q, manifold.localNormal);
					Vec2 planePoint = Utilities.Mul(xfA, manifold.localPoint);

					points.Clear();
					for (int i = 0; i < manifold.points.Count(); ++i)
					{
						Vec2 clipPoint = Utilities.Mul(xfB, manifold.points[i].localPoint);
						Vec2 cA = clipPoint + (radiusA - Utilities.Dot(clipPoint - planePoint, normal)) * normal;
						Vec2 cB = clipPoint - radiusB * normal;
						points.Add(0.5f * (cA + cB));
					}
				}
				break;

			case Manifold.ManifoldType.e_faceB:
				{
					normal = Utilities.Mul(xfB.q, manifold.localNormal);
					Vec2 planePoint = Utilities.Mul(xfB, manifold.localPoint);

					points.Clear();
					for (int i = 0; i < manifold.points.Count(); ++i)
					{
						Vec2 clipPoint = Utilities.Mul(xfA, manifold.points[i].localPoint);
						Vec2 cB = clipPoint + (radiusB - Utilities.Dot(clipPoint - planePoint, normal)) * normal;
						Vec2 cA = clipPoint - radiusA * normal;
						points.Add(0.5f * (cA + cB));
					}

					// Ensure normal points from A to B.
					normal = -normal;
				}
				break;
			}
		}

		public Vec2 normal;							///< world vector pointing from A to B
		public List<Vec2> points;	///< world contact point (point of intersection)
	}

	/// This is used for determining the state of contact points.
	public enum PointState
	{
		_nullState,		///< point does not exist
		_addState,		///< point was added in the update
		_persistState,	///< point persisted across the update
		_removeState		///< point was removed in the update
	}

	/// Used for computing contact manifolds.
	public struct ClipVertex
	{
		public Vec2 v;
		public ContactID id;
	}

	/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	public struct RayCastInput
	{
		public Vec2 p1, p2;
		public float maxFraction;
	}

	/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
	/// come from RayCastInput.
	public struct RayCastOutput
	{
		public Vec2 normal;
		public float fraction;
	}

	/// An axis aligned bounding box.
	public struct AABB
	{
		/// Verify that the bounds are sorted.
		public bool IsValid(){
			Vec2 d = upperBound - lowerBound;
			bool valid = d.X >= 0.0f && d.Y >= 0.0f;
			valid = valid && lowerBound.IsValid() && upperBound.IsValid();
			return valid;
		}

		/// Get the center of the AABB.
		public Vec2 GetCenter()
		{
			return 0.5f * (lowerBound + upperBound);
		}

		/// Get the extents of the AABB (half-widths).
		public Vec2 GetExtents()
		{
			return 0.5f * (upperBound - lowerBound);
		}

		/// Get the perimeter length
		public float GetPerimeter()
		{
			float wx = upperBound.X - lowerBound.X;
			float wy = upperBound.Y - lowerBound.Y;
			return 2.0f * (wx + wy);
		}

		/// Combine an AABB into this one.
		public void Combine(AABB aabb)
		{
			throw new NotImplementedException();
			//lowerBound = Math.Min(lowerBound, aabb.lowerBound);
			//upperBound = Math.Max(upperBound, aabb.upperBound);
		}

		/// Combine two AABBs into this one.
		public void Combine(AABB aabb1, AABB aab)
		{
			lowerBound = Utilities.Min(aabb1.lowerBound, aab.lowerBound);
			upperBound = Utilities.Max(aabb1.upperBound, aab.upperBound);
		}

		/// Does this aabb contain the provided AABB.
		public bool Contains(AABB aabb)
		{
			bool result = true;
			result = result && lowerBound.X <= aabb.lowerBound.X;
			result = result && lowerBound.Y <= aabb.lowerBound.Y;
			result = result && aabb.upperBound.X <= upperBound.X;
			result = result && aabb.upperBound.Y <= upperBound.Y;
			return result;
		}

		public bool RayCast(out RayCastOutput output, RayCastInput input) {
			float tmin = -Single.MaxValue;
			float tmax = Single.MaxValue;

			Vec2 p = input.p1;
			Vec2 d = input.p2 - input.p1;
			Vec2 absD = Utilities.Abs(d);
			output = new RayCastOutput();

			Vec2 normal = new Vec2();

			for (int i = 0; i < 2; ++i) {
				if (Math.Abs(i) < Single.Epsilon) {
					// Parallel.
					if (p[i] < lowerBound[i] || upperBound[i] < p[i]) {
						return false;
					}
				} else {
					float inv_d = 1.0f / d[i];
					float t1 = (lowerBound[i] - p[i]) * inv_d;
					float t2 = (upperBound[i] - p[i]) * inv_d;

					// Sign of the normal vector.
					float s = -1.0f;

					if (t1 > t2) {
						float temp = t1;
						t1 = t2;
						t2 = temp;
						s = 1.0f;
					}

					// Push the min up
					if (t1 > tmin) {
						normal.SetZero();
						normal[i] = s;
						tmin = t1;
					}

					// Pull the max down
					tmax = Math.Min(tmax, t2);

					if (tmin > tmax) {
						return false;
					}
				}
			}

			// Does the ray start inside the box?
			// Does the ray intersect beyond the max fraction?
			if (tmin < 0.0f || input.maxFraction < tmin) {
				return false;
			}

			// Intersection.
			output.fraction = tmin;
			output.normal = normal;
			return true;
		}

		public Vec2 lowerBound;	///< the lower vertex
		public Vec2 upperBound;	///< the upper vertex
	}
}
