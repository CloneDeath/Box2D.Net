using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2PositionSolverManifold
	{
		public void Initialize(b2ContactPositionConstraint pc, b2Transform xfA, b2Transform xfB, int index)
		{
			Utilities.Assert(pc.pointCount > 0);

			switch (pc.type)
			{
			case b2Manifold.ManifoldType.e_circles:
				{
					b2Vec2 pointA = Utilities.b2Mul(xfA, pc.localPoint);
					b2Vec2 pointB = Utilities.b2Mul(xfB, pc.localPoints[0]);
					normal = pointB - pointA;
					normal.Normalize();
					point = 0.5f * (pointA + pointB);
					separation = Utilities.b2Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
				}
				break;

			case b2Manifold.ManifoldType.e_faceA:
				{
					normal = Utilities.b2Mul(xfA.q, pc.localNormal);
					b2Vec2 planePoint = Utilities.b2Mul(xfA, pc.localPoint);

					b2Vec2 clipPoint = Utilities.b2Mul(xfB, pc.localPoints[index]);
					separation = Utilities.b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
					point = clipPoint;
				}
				break;

			case b2Manifold.ManifoldType.e_faceB:
				{
					normal = Utilities.b2Mul(xfB.q, pc.localNormal);
					b2Vec2 planePoint = Utilities.b2Mul(xfB, pc.localPoint);

					b2Vec2 clipPoint = Utilities.b2Mul(xfA, pc.localPoints[index]);
					separation = Utilities.b2Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
					point = clipPoint;

					// Ensure normal points from A to B
					normal = -normal;
				}
				break;
			}
		}

		public b2Vec2 normal;
		public b2Vec2 point;
		public float separation;
	};
}
