using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Box2D {
	public class b2Collision {
		public const byte b2_nullFeature = byte.MaxValue;
		
		/// Compute the collision manifold between two circles.
		public void b2CollideCircles(b2Manifold* manifold,
							  const b2CircleShape* circleA, const b2Transform& xfA,
							  const b2CircleShape* circleB, const b2Transform& xfB);

		/// Compute the collision manifold between a polygon and a circle.
		public void b2CollidePolygonAndCircle(b2Manifold* manifold,
									   const b2PolygonShape* polygonA, const b2Transform& xfA,
									   const b2CircleShape* circleB, const b2Transform& xfB);

		/// Compute the collision manifold between two polygons.
		public void b2CollidePolygons(b2Manifold* manifold,
							   const b2PolygonShape* polygonA, const b2Transform& xfA,
							   const b2PolygonShape* polygonB, const b2Transform& xfB);

		/// Compute the collision manifold between an edge and a circle.
		public void b2CollideEdgeAndCircle(b2Manifold* manifold,
									   const b2EdgeShape* polygonA, const b2Transform& xfA,
									   const b2CircleShape* circleB, const b2Transform& xfB);

		/// Compute the collision manifold between an edge and a circle.
		public void b2CollideEdgeAndPolygon(b2Manifold* manifold,
									   const b2EdgeShape* edgeA, const b2Transform& xfA,
									   const b2PolygonShape* circleB, const b2Transform& xfB);

		/// Clipping for contact manifolds.
		public int b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
									const b2Vec2& normal, float offset, int vertexIndexA);

		/// Determine if two generic shapes overlap.
		public bool b2TestOverlap(	const b2Shape* shapeA, int indexA,
							const b2Shape* shapeB, int indexB,
							const b2Transform& xfA, const b2Transform& xfB){
			b2Vec2 d1, d2;
			d1 = b.lowerBound - a.upperBound;
			d2 = a.lowerBound - b.upperBound;

			if (d1.x > 0.0f || d1.y > 0.0f)
				return false;

			if (d2.x > 0.0f || d2.y > 0.0f)
				return false;

			return true;					
		}
		
		/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
		/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
		public void b2GetPointStates(b2PointState state1[b2Settings.b2_maxManifoldPoints], b2PointState state2[b2Settings.b2_maxManifoldPoints],
							  const b2Manifold* manifold1, const b2Manifold* manifold2);
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
		public enum ManifoldType
		{
			e_circles,
			e_faceA,
			e_faceB
		};

		public b2ManifoldPoint[] points = new b2ManifoldPoint[Settings.b2Settings.b2_maxManifoldPoints];	///< the points of contact
		public b2Vec2 localNormal;								///< not use for Type::e_points
		public b2Vec2 localPoint;								///< usage depends on manifold type
		public ManifoldType type;
		public int pointCount;								///< the number of manifold points
	}

	/// This is used to compute the current state of a contact manifold.
	public struct b2WorldManifold
	{
		/// Evaluate the manifold with supplied transforms. This assumes
		/// modest motion from the original state. This does not change the
		/// point count, impulses, etc. The radii must come from the shapes
		/// that generated the manifold.
		public void Initialize(const b2Manifold* manifold,
								b2Transform xfA, float radiusA,
								b2Transform xfB, float radiusB);

		public b2Vec2 normal;							///< world vector pointing from A to B
		public b2Vec2 points[b2Settings.b2_maxManifoldPoints];	///< world contact point (point of intersection)
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
			lowerBound = Math.Min(lowerBound, aabb.lowerBound);
			upperBound = b2Max(upperBound, aabb.upperBound);
		}

		/// Combine two AABBs into this one.
		public void Combine(b2AABB aabb1, b2AABB aabb2)
		{
			lowerBound = Math.Min(aabb1.lowerBound, aabb2.lowerBound);
			upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
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

		public bool RayCast(out b2RayCastOutput output, b2RayCastInput input);

		public b2Vec2 lowerBound;	///< the lower vertex
		public b2Vec2 upperBound;	///< the upper vertex
	}
}
