using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Box2D {
	public class b2Collision {
		public const byte b2_nullFeature = byte.MaxValue;
		
		/// Compute the collision manifold between two circles.
		public static void b2CollideCircles(out b2Manifold manifold,
							  b2CircleShape circleA, b2Transform xfA,
							  b2CircleShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
			//manifold.pointCount = 0;

			//b2Vec2 pA = Utilities.b2Mul(xfA, circleA.m_p);
			//b2Vec2 pB = Utilities.b2Mul(xfB, circleB.m_p);

			//b2Vec2 d = pB - pA;
			//float distSqr = Utilities.b2Dot(d, d);
			//float rA = circleA.m_radius, rB = circleB.m_radius;
			//float radius = rA + rB;
			//if (distSqr > radius * radius)
			//{
			//    return;
			//}

			//manifold.type = b2Manifold::e_circles;
			//manifold.localPoint = circleA.m_p;
			//manifold.localNormal.SetZero();
			//manifold.pointCount = 1;

			//manifold.points[0].localPoint = circleB.m_p;
			//manifold.points[0].id.key = 0;
		}

		/// Compute the collision manifold between a polygon and a circle.
		public static void b2CollidePolygonAndCircle(out b2Manifold manifold,
									   b2PolygonShape polygonA, b2Transform xfA,
									   b2CircleShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
			//manifold.pointCount = 0;

			//// Compute circle position in the frame of the polygon.
			//b2Vec2 c = Utilities.b2Mul(xfB, circleB.m_p);
			//b2Vec2 cLocal = Utilities.b2MulT(xfA, c);

			//// Find the min separating edge.
			//int normalIndex = 0;
			//float separation = -Single.MaxValue;
			//float radius = polygonA.m_radius + circleB.m_radius;
			//int vertexCount = polygonA.m_count;
			//const b2Vec2* vertices = polygonA.m_vertices;
			//const b2Vec2* normals = polygonA.m_normals;

			//for (int i = 0; i < vertexCount; ++i)
			//{
			//    float s = Utilities.b2Dot(normals[i], cLocal - vertices[i]);

			//    if (s > radius)
			//    {
			//        // Early out.
			//        return;
			//    }

			//    if (s > separation)
			//    {
			//        separation = s;
			//        normalIndex = i;
			//    }
			//}

			//// Vertices that subtend the incident face.
			//int vertIndex1 = normalIndex;
			//int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
			//b2Vec2 v1 = vertices[vertIndex1];
			//b2Vec2 v2 = vertices[vertIndex2];

			//// If the center is inside the polygon ...
			//if (separation < Single.Epsilon)
			//{
			//    manifold.pointCount = 1;
			//    manifold.type = b2Manifold::e_faceA;
			//    manifold.localNormal = normals[normalIndex];
			//    manifold.localPoint = 0.5f * (v1 + v2);
			//    manifold.points[0].localPoint = circleB.m_p;
			//    manifold.points[0].id.key = 0;
			//    return;
			//}

			//// Compute barycentric coordinates
			//float u1 = Utilities.b2Dot(cLocal - v1, v2 - v1);
			//float u2 = Utilities.b2Dot(cLocal - v2, v1 - v2);
			//if (u1 <= 0.0f)
			//{
			//    if (b2DistanceSquared(cLocal, v1) > radius * radius)
			//    {
			//        return;
			//    }

			//    manifold.pointCount = 1;
			//    manifold.type = b2Manifold::e_faceA;
			//    manifold.localNormal = cLocal - v1;
			//    manifold.localNormal.Normalize();
			//    manifold.localPoint = v1;
			//    manifold.points[0].localPoint = circleB.m_p;
			//    manifold.points[0].id.key = 0;
			//}
			//else if (u2 <= 0.0f)
			//{
			//    if (b2DistanceSquared(cLocal, v2) > radius * radius)
			//    {
			//        return;
			//    }

			//    manifold.pointCount = 1;
			//    manifold.type = b2Manifold::e_faceA;
			//    manifold.localNormal = cLocal - v2;
			//    manifold.localNormal.Normalize();
			//    manifold.localPoint = v2;
			//    manifold.points[0].localPoint = circleB.m_p;
			//    manifold.points[0].id.key = 0;
			//}
			//else
			//{
			//    b2Vec2 faceCenter = 0.5f * (v1 + v2);
			//    float separation = Utilities.b2Dot(cLocal - faceCenter, normals[vertIndex1]);
			//    if (separation > radius)
			//    {
			//        return;
			//    }

			//    manifold.pointCount = 1;
			//    manifold.type = b2Manifold::e_faceA;
			//    manifold.localNormal = normals[vertIndex1];
			//    manifold.localPoint = faceCenter;
			//    manifold.points[0].localPoint = circleB.m_p;
			//    manifold.points[0].id.key = 0;
			//}
		}

		// Find the separation between poly1 and poly2 for a give edge normal on poly1.
		static float b2EdgeSeparation(b2PolygonShape poly1, b2Transform xf1, int edge1,
									  b2PolygonShape poly2, b2Transform xf2)
		{
			b2Vec2[] vertices1 = poly1.m_vertices;
			b2Vec2[] normals1 = poly1.m_normals;

			int count2 = poly2.m_count;
			b2Vec2[] vertices2 = poly2.m_vertices;

			Utilities.Assert(0 <= edge1 && edge1 < poly1.m_count);

			// Convert normal from poly1's frame into poly2's frame.
			b2Vec2 normal1World = Utilities.b2Mul(xf1.q, normals1[edge1]);
			b2Vec2 normal1 = Utilities.b2MulT(xf2.q, normal1World);

			// Find support vertex on poly2 for -normal.
			int index = 0;
			float minDot = Single.MaxValue;

			for (int i = 0; i < count2; ++i)
			{
				float dot = Utilities.b2Dot(vertices2[i], normal1);
				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

			b2Vec2 v1 = Utilities.b2Mul(xf1, vertices1[edge1]);
			b2Vec2 v2 = Utilities.b2Mul(xf2, vertices2[index]);
			float separation = Utilities.b2Dot(v2 - v1, normal1World);
			return separation;
		}


		// Find the max separation between poly1 and poly2 using edge normals from poly1.
		static float b2FindMaxSeparation(out int edgeIndex,
										 b2PolygonShape poly1, b2Transform xf1,
										 b2PolygonShape poly2, b2Transform xf2)
		{
			int count1 = poly1.m_count;
			b2Vec2[] normals1 = poly1.m_normals;

			// Vector pointing from the centroid of poly1 to the centroid of poly2.
			b2Vec2 d = Utilities.b2Mul(xf2, poly2.m_centroid) - Utilities.b2Mul(xf1, poly1.m_centroid);
			b2Vec2 dLocal1 = Utilities.b2MulT(xf1.q, d);

			// Find edge normal on poly1 that has the largest projection onto d.
			int edge = 0;
			float maxDot = -Single.MaxValue;
			for (int i = 0; i < count1; ++i)
			{
				float dot = Utilities.b2Dot(normals1[i], dLocal1);
				if (dot > maxDot)
				{
					maxDot = dot;
					edge = i;
				}
			}

			// Get the separation for the edge normal.
			float s = b2EdgeSeparation(poly1, xf1, edge, poly2, xf2);

			// Check the separation for the previous edge normal.
			int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
			float sPrev = b2EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

			// Check the separation for the next edge normal.
			int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
			float sNext = b2EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

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

				s = b2EdgeSeparation(poly1, xf1, edge, poly2, xf2);

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

		static void b2FindIncidentEdge(b2ClipVertex[/*2*/] c,
							 b2PolygonShape poly1, b2Transform xf1, int edge1,
							 b2PolygonShape poly2, b2Transform xf2)
		{
			b2Vec2[] normals1 = poly1.m_normals;

			int count2 = poly2.m_count;
			b2Vec2[] vertices2 = poly2.m_vertices;
			b2Vec2[] normals2 = poly2.m_normals;

			Utilities.Assert(0 <= edge1 && edge1 < poly1.m_count);

			// Get the normal of the reference edge in poly2's frame.
			b2Vec2 normal1 = Utilities.b2MulT(xf2.q, Utilities.b2Mul(xf1.q, normals1[edge1]));

			// Find the incident edge on poly2.
			int index = 0;
			float minDot = Single.MaxValue;
			for (int i = 0; i < count2; ++i)
			{
				float dot = Utilities.b2Dot(normal1, normals2[i]);
				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

			// Build the clip vertices for the incident edge.
			int i1 = index;
			int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

			c[0].v = Utilities.b2Mul(xf2, vertices2[i1]);
			c[0].id.cf.indexA = (byte)edge1;
			c[0].id.cf.indexB = (byte)i1;
			c[0].id.cf.typeA = b2ContactFeature.FeatureType.e_face;
			c[0].id.cf.typeB = b2ContactFeature.FeatureType.e_vertex;

			c[1].v = Utilities.b2Mul(xf2, vertices2[i2]);
			c[1].id.cf.indexA = (byte)edge1;
			c[1].id.cf.indexB = (byte)i2;
			c[1].id.cf.typeA = b2ContactFeature.FeatureType.e_face;
			c[1].id.cf.typeB = b2ContactFeature.FeatureType.e_vertex;
		}

		// Find edge normal of max separation on A - return if separating axis is found
		// Find edge normal of max separation on B - return if separation axis is found
		// Choose reference edge as min(minA, minB)
		// Find incident edge
		// Clip

		// The normal points from 1 to 2
		/// Compute the collision manifold between two polygons.
		public static void b2CollidePolygons(out b2Manifold manifold,
							   b2PolygonShape polyA, b2Transform xfA,
							   b2PolygonShape polyB, b2Transform xfB) {
			manifold = new b2Manifold();
			float totalRadius = polyA.m_radius + polyB.m_radius;

			int edgeA = 0;
			float separationA = b2FindMaxSeparation(out edgeA, polyA, xfA, polyB, xfB);
			if (separationA > totalRadius)
			    return;

			int edgeB = 0;
			float separationB = b2FindMaxSeparation(out edgeB, polyB, xfB, polyA, xfA);
			if (separationB > totalRadius)
			    return;

			b2PolygonShape poly1;	// reference polygon
			b2PolygonShape poly2;	// incident polygon
			b2Transform xf1, xf2;
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
			    manifold.type = b2Manifold.ManifoldType.e_faceB;
			    flip = true;
			}
			else
			{
			    poly1 = polyA;
			    poly2 = polyB;
			    xf1 = xfA;
			    xf2 = xfB;
			    edge1 = edgeA;
			    manifold.type = b2Manifold.ManifoldType.e_faceA;
			    flip = false;
			}

			b2ClipVertex[] incidentEdge = new b2ClipVertex[2];
			b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

			int count1 = poly1.m_count;
			b2Vec2[] vertices1 = poly1.m_vertices;

			int iv1 = edge1;
			int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

			b2Vec2 v11 = vertices1[iv1];
			b2Vec2 v12 = vertices1[iv2];

			b2Vec2 localTangent = v12 - v11;
			localTangent.Normalize();
	
			b2Vec2 localNormal = Utilities.b2Cross(localTangent, 1.0f);
			b2Vec2 planePoint = 0.5f * (v11 + v12);

			b2Vec2 tangent = Utilities.b2Mul(xf1.q, localTangent);
			b2Vec2 normal = Utilities.b2Cross(tangent, 1.0f);
	
			v11 = Utilities.b2Mul(xf1, v11);
			v12 = Utilities.b2Mul(xf1, v12);

			// Face offset.
			float frontOffset = Utilities.b2Dot(normal, v11);

			// Side offsets, extended by polytope skin thickness.
			float sideOffset1 = -Utilities.b2Dot(tangent, v11) + totalRadius;
			float sideOffset2 = Utilities.b2Dot(tangent, v12) + totalRadius;

			// Clip incident edge against extruded edge1 side edges.
			b2ClipVertex[] clipPoints1 = new b2ClipVertex[2];
			b2ClipVertex[] clipPoints2 = new b2ClipVertex[2];
			int np;

			// Clip to box side 1
			np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

			if (np < 2)
			    return;

			// Clip to negative box side 1
			np = b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

			if (np < 2)
			{
			    return;
			}

			// Now clipPoints2 contains the clipped points.
			manifold.localNormal = localNormal;
			manifold.localPoint = planePoint;

			manifold.points.Clear();
			for (int i = 0; i < b2Settings.b2_maxManifoldPoints; ++i)
			{
			    float separation = Utilities.b2Dot(normal, clipPoints2[i].v) - frontOffset;

			    if (separation <= totalRadius)
			    {
					b2ManifoldPoint cp = new b2ManifoldPoint();
			        cp.localPoint = Utilities.b2MulT(xf2, clipPoints2[i].v);
			        cp.id = clipPoints2[i].id;
			        if (flip)
			        {
			            // Swap features
			            b2ContactFeature cf = cp.id.cf;
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
		public static void b2CollideEdgeAndCircle(out b2Manifold manifold,
									   b2EdgeShape polygonA, b2Transform xfA,
									   b2CircleShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
		}

		/// Compute the collision manifold between an edge and a circle.
		public static void b2CollideEdgeAndPolygon(out b2Manifold manifold,
									   b2EdgeShape edgeA, b2Transform xfA,
									   b2PolygonShape polygonB, b2Transform xfB) {
			b2EPCollider collider = new b2EPCollider();
			collider.Collide(out manifold, edgeA, xfA, polygonB, xfB);
		}

		/// Clipping for contact manifolds.
		// Sutherland-Hodgman clipping.
		public static int b2ClipSegmentToLine(b2ClipVertex[/*2*/] vOut, b2ClipVertex[/*2*/] vIn,
									b2Vec2 normal, float offset, int vertexIndexA){
			// Start with no output points
			int numOut = 0;

			// Calculate the distance of end points to the line
			float distance0 = Utilities.b2Dot(normal, vIn[0].v) - offset;
			float distance1 = Utilities.b2Dot(normal, vIn[1].v) - offset;

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
				vOut[numOut].id.cf.typeA = b2ContactFeature.FeatureType.e_vertex;
				vOut[numOut].id.cf.typeB = b2ContactFeature.FeatureType.e_face;
				++numOut;
			}

			return numOut;
		}

		/// Determine if two generic shapes overlap.
		public static bool b2TestOverlap(b2Shape shapeA, int indexA,
							b2Shape shapeB, int indexB,
							b2Transform xfA, b2Transform xfB){

			b2DistanceInput input = new b2DistanceInput();
			input.proxyA.Set(shapeA, indexA);
			input.proxyB.Set(shapeB, indexB);
			input.transformA = xfA;
			input.transformB = xfB;
			input.useRadii = true;

			b2SimplexCache cache = new b2SimplexCache();
			cache.count = 0;

			b2DistanceOutput output;

			b2Distance.Distance(out output, cache, input);

			return output.distance < 10.0f * Single.Epsilon;
		}
		
		/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
		/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
		public static void b2GetPointStates(b2PointState[/*b2Settings.b2_maxManifoldPoints*/] state1, b2PointState[/*b2Settings.b2_maxManifoldPoints*/] state2,
							  b2Manifold manifold1, b2Manifold manifold2) {
			throw new NotImplementedException();
			//for (int i = 0; i < b2Settings.b2_maxManifoldPoints; ++i)
			//{
			//    state1[i] = b2_nullState;
			//    state2[i] = b2_nullState;
			//}

			//// Detect persists and removes.
			//for (int i = 0; i < manifold1.pointCount; ++i)
			//{
			//    b2ContactID id = manifold1.points[i].id;

			//    state1[i] = b2_removeState;

			//    for (int j = 0; j < manifold2.pointCount; ++j)
			//    {
			//        if (manifold2.points[j].id.key == id.key)
			//        {
			//            state1[i] = b2_persistState;
			//            break;
			//        }
			//    }
			//}

			//// Detect persists and adds.
			//for (int i = 0; i < manifold2.pointCount; ++i)
			//{
			//    b2ContactID id = manifold2.points[i].id;

			//    state2[i] = b2_addState;

			//    for (int j = 0; j < manifold1.pointCount; ++j)
			//    {
			//        if (manifold1.points[j].id.key == id.key)
			//        {
			//            state2[i] = b2_persistState;
			//            break;
			//        }
			//    }
			//}
		}

		public static bool b2TestOverlap(b2AABB a, b2AABB b) {
			b2Vec2 d1, d2;
			d1 = b.lowerBound - a.upperBound;
			d2 = a.lowerBound - b.upperBound;

			if (d1.x > 0.0f || d1.y > 0.0f)
				return false;

			if (d2.x > 0.0f || d2.y > 0.0f)
				return false;

			return true;					
		}
		//{
		//    return (a.lowerBound.x <= b.upperBound.x) && (a.lowerBound.y <= b.upperBound.y) &&
		//           (a.upperBound.x >= b.lowerBound.x) && (a.upperBound.y >= b.lowerBound.y);
		//}
	}

	/// The features that intersect to form the contact point
	/// This must be 4 bytes or less.
	public struct b2ContactFeature
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
	public struct b2ContactID
	{
		public b2ContactFeature cf;
		public uint key {
			get {
				return (uint)((cf.indexA << 24) & (cf.indexB << 16) & ((byte)cf.typeA << 8) & ((byte)cf.typeB));
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
	public class b2ManifoldPoint //was struct
	{
		public b2Vec2 localPoint;		///< usage depends on manifold type
		public float normalImpulse;	///< the non-penetration impulse
		public float tangentImpulse;	///< the friction impulse
		public b2ContactID id;			///< uniquely identifies a contact point between two shapes
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
	public class b2Manifold
	{
		public b2Manifold() {
			points = new List<b2ManifoldPoint>();
			localNormal = new b2Vec2();
			localPoint = new b2Vec2();
		}

		public b2Manifold(ManifoldType type) {
			points = new List<b2ManifoldPoint>();
			localNormal = new b2Vec2(0, 0);
			localPoint = new b2Vec2(0, 0);
			this.type = type;
		}

		public enum ManifoldType
		{
			e_circles,
			e_faceA,
			e_faceB
		};

		public List<b2ManifoldPoint> points;	///< the points of contact
		public b2Vec2 localNormal;								///< not use for Type::e_points
		public b2Vec2 localPoint;								///< usage depends on manifold type
		public ManifoldType type;
		//public int pointCount;								///< the number of manifold points
	}

	/// This is used to compute the current state of a contact manifold.
	public class b2WorldManifold //was struct
	{
		public b2WorldManifold() {
			points = new List<b2Vec2>();
		}
		public b2WorldManifold(b2Vec2 normal) {
			points = new List<b2Vec2>();
			this.normal = normal;
		}

		/// Evaluate the manifold with supplied transforms. This assumes
		/// modest motion from the original state. This does not change the
		/// point count, impulses, etc. The radii must come from the shapes
		/// that generated the manifold.
		public void Initialize(b2Manifold manifold,
								b2Transform xfA, float radiusA,
								b2Transform xfB, float radiusB){
			if (manifold.points.Count() == 0)
			{
				return;
			}

			switch (manifold.type)
			{
			case b2Manifold.ManifoldType.e_circles:
				{
					normal.Set(1.0f, 0.0f);
					b2Vec2 pointA = Utilities.b2Mul(xfA, manifold.localPoint);
					b2Vec2 pointB = Utilities.b2Mul(xfB, manifold.points[0].localPoint);
					if (Utilities.b2DistanceSquared(pointA, pointB) > Single.Epsilon * Single.Epsilon)
					{
						normal = pointB - pointA;
						normal.Normalize();
					}

					b2Vec2 cA = pointA + radiusA * normal;
					b2Vec2 cB = pointB - radiusB * normal;
					points[0] = 0.5f * (cA + cB);
				}
				break;

			case b2Manifold.ManifoldType.e_faceA:
				{
					normal = Utilities.b2Mul(xfA.q, manifold.localNormal);
					b2Vec2 planePoint = Utilities.b2Mul(xfA, manifold.localPoint);

					points.Clear();
					for (int i = 0; i < manifold.points.Count(); ++i)
					{
						b2Vec2 clipPoint = Utilities.b2Mul(xfB, manifold.points[i].localPoint);
						b2Vec2 cA = clipPoint + (radiusA - Utilities.b2Dot(clipPoint - planePoint, normal)) * normal;
						b2Vec2 cB = clipPoint - radiusB * normal;
						points.Add(0.5f * (cA + cB));
					}
				}
				break;

			case b2Manifold.ManifoldType.e_faceB:
				{
					normal = Utilities.b2Mul(xfB.q, manifold.localNormal);
					b2Vec2 planePoint = Utilities.b2Mul(xfB, manifold.localPoint);

					points.Clear();
					for (int i = 0; i < manifold.points.Count(); ++i)
					{
						b2Vec2 clipPoint = Utilities.b2Mul(xfA, manifold.points[i].localPoint);
						b2Vec2 cB = clipPoint + (radiusB - Utilities.b2Dot(clipPoint - planePoint, normal)) * normal;
						b2Vec2 cA = clipPoint - radiusA * normal;
						points.Add(0.5f * (cA + cB));
					}

					// Ensure normal points from A to B.
					normal = -normal;
				}
				break;
			}
		}

		public b2Vec2 normal;							///< world vector pointing from A to B
		public List<b2Vec2> points;	///< world contact point (point of intersection)
	}

	/// This is used for determining the state of contact points.
	public enum b2PointState
	{
		b2_nullState,		///< point does not exist
		b2_addState,		///< point was added in the update
		b2_persistState,	///< point persisted across the update
		b2_removeState		///< point was removed in the update
	}

	/// Used for computing contact manifolds.
	public struct b2ClipVertex
	{
		public b2Vec2 v;
		public b2ContactID id;
	}

	/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	public struct b2RayCastInput
	{
		public b2Vec2 p1, p2;
		public float maxFraction;
	}

	/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
	/// come from b2RayCastInput.
	public struct b2RayCastOutput
	{
		public b2Vec2 normal;
		public float fraction;
	}

	/// An axis aligned bounding box.
	public struct b2AABB
	{
		/// Verify that the bounds are sorted.
		public bool IsValid(){
			b2Vec2 d = upperBound - lowerBound;
			bool valid = d.x >= 0.0f && d.y >= 0.0f;
			valid = valid && lowerBound.IsValid() && upperBound.IsValid();
			return valid;
		}

		/// Get the center of the AABB.
		public b2Vec2 GetCenter()
		{
			return 0.5f * (lowerBound + upperBound);
		}

		/// Get the extents of the AABB (half-widths).
		public b2Vec2 GetExtents()
		{
			return 0.5f * (upperBound - lowerBound);
		}

		/// Get the perimeter length
		public float GetPerimeter()
		{
			float wx = upperBound.x - lowerBound.x;
			float wy = upperBound.y - lowerBound.y;
			return 2.0f * (wx + wy);
		}

		/// Combine an AABB into this one.
		public void Combine(b2AABB aabb)
		{
			throw new NotImplementedException();
			//lowerBound = Math.Min(lowerBound, aabb.lowerBound);
			//upperBound = Math.Max(upperBound, aabb.upperBound);
		}

		/// Combine two AABBs into this one.
		public void Combine(b2AABB aabb1, b2AABB aabb2)
		{
			lowerBound = Utilities.Min(aabb1.lowerBound, aabb2.lowerBound);
			upperBound = Utilities.Max(aabb1.upperBound, aabb2.upperBound);
		}

		/// Does this aabb contain the provided AABB.
		public bool Contains(b2AABB aabb)
		{
			bool result = true;
			result = result && lowerBound.x <= aabb.lowerBound.x;
			result = result && lowerBound.y <= aabb.lowerBound.y;
			result = result && aabb.upperBound.x <= upperBound.x;
			result = result && aabb.upperBound.y <= upperBound.y;
			return result;
		}

		public bool RayCast(out b2RayCastOutput output, b2RayCastInput input) {
			throw new NotImplementedException();
		}

		public b2Vec2 lowerBound;	///< the lower vertex
		public b2Vec2 upperBound;	///< the upper vertex
	}
}
