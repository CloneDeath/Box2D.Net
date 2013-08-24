using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Box2D {
	public class b2Collision {
		public const byte b2_nullFeature = byte.MaxValue;
		
		/// Compute the collision manifold between two circles.
		public static void b2CollideCircles(b2Manifold manifold,
							  b2CircleShape circleA, b2Transform xfA,
							  b2CircleShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
		}

		/// Compute the collision manifold between a polygon and a circle.
		public static void b2CollidePolygonAndCircle(b2Manifold manifold,
									   b2PolygonShape polygonA, b2Transform xfA,
									   b2CircleShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
		}

		/// Compute the collision manifold between two polygons.
		public static void b2CollidePolygons(b2Manifold manifold,
							   b2PolygonShape polygonA, b2Transform xfA,
							   b2PolygonShape polygonB, b2Transform xfB) {
			throw new NotImplementedException();
		}

		/// Compute the collision manifold between an edge and a circle.
		public static void b2CollideEdgeAndCircle(b2Manifold manifold,
									   b2EdgeShape polygonA, b2Transform xfA,
									   b2CircleShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
		}

		/// Compute the collision manifold between an edge and a circle.
		public static void b2CollideEdgeAndPolygon(b2Manifold manifold,
									   b2EdgeShape edgeA, b2Transform xfA,
									   b2PolygonShape circleB, b2Transform xfB) {
			throw new NotImplementedException();
		}

		/// Clipping for contact manifolds.
		public static int b2ClipSegmentToLine(b2ClipVertex[/*2*/] vOut, b2ClipVertex[/*2*/] vIn,
									b2Vec2 normal, float offset, int vertexIndexA) {
			throw new NotImplementedException();
		}

		/// Determine if two generic shapes overlap.
		public static bool b2TestOverlap(b2Shape shapeA, int indexA,
							b2Shape shapeB, int indexB,
							b2Transform xfA, b2Transform xfB){
			throw new NotImplementedException();								
			//b2Vec2 d1, d2;
			//d1 = b.lowerBound - a.upperBound;
			//d2 = a.lowerBound - b.upperBound;

			//if (d1.x > 0.0f || d1.y > 0.0f)
			//    return false;

			//if (d2.x > 0.0f || d2.y > 0.0f)
			//    return false;

			//return true;					
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

		internal static bool b2TestOverlap(b2AABB a, b2AABB b) {
			return (a.lowerBound.x <= b.upperBound.x) && (a.lowerBound.y <= b.upperBound.y) &&
				   (a.upperBound.x >= b.lowerBound.x) && (a.upperBound.y >= b.lowerBound.y);
		}
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
	public struct b2ManifoldPoint
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
	public struct b2Manifold
	{
		public b2Manifold(ManifoldType type) {
			points = new b2ManifoldPoint[b2Settings.b2_maxManifoldPoints];
			localNormal = new b2Vec2(0, 0);
			localPoint = new b2Vec2(0, 0);
			this.type = type;
			pointCount = 0;
		}

		public enum ManifoldType
		{
			e_circles,
			e_faceA,
			e_faceB
		};

		public b2ManifoldPoint[] points;	///< the points of contact
		public b2Vec2 localNormal;								///< not use for Type::e_points
		public b2Vec2 localPoint;								///< usage depends on manifold type
		public ManifoldType type;
		public int pointCount;								///< the number of manifold points
	}

	/// This is used to compute the current state of a contact manifold.
	public struct b2WorldManifold
	{
		public b2WorldManifold(b2Vec2 normal) {
			points = new b2Vec2[b2Settings.b2_maxManifoldPoints];
			this.normal = normal;
		}

		/// Evaluate the manifold with supplied transforms. This assumes
		/// modest motion from the original state. This does not change the
		/// point count, impulses, etc. The radii must come from the shapes
		/// that generated the manifold.
		public void Initialize(b2Manifold manifold,
								b2Transform xfA, float radiusA,
								b2Transform xfB, float radiusB) {
			throw new NotImplementedException();
		}

		public b2Vec2 normal;							///< world vector pointing from A to B
		public b2Vec2[] points;	///< world contact point (point of intersection)
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
			//upperBound = b2Max(upperBound, aabb.upperBound);
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
