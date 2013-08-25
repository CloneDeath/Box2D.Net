using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct PositionSolverManifold
	{
		public void Initialize(ContactPositionConstraint pc, Transform xfA, Transform xfB, int index)
		{
			Utilities.Assert(pc.pointCount > 0);

			switch (pc.type)
			{
			case Manifold.ManifoldType.e_circles:
				{
					Vec2 pointA = Utilities.Mul(xfA, pc.localPoint);
					Vec2 pointB = Utilities.Mul(xfB, pc.localPoints[0]);
					normal = pointB - pointA;
					normal.Normalize();
					point = 0.5f * (pointA + pointB);
					separation = Utilities.Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
				}
				break;

			case Manifold.ManifoldType.e_faceA:
				{
					normal = Utilities.Mul(xfA.q, pc.localNormal);
					Vec2 planePoint = Utilities.Mul(xfA, pc.localPoint);

					Vec2 clipPoint = Utilities.Mul(xfB, pc.localPoints[index]);
					separation = Utilities.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
					point = clipPoint;
				}
				break;

			case Manifold.ManifoldType.e_faceB:
				{
					normal = Utilities.Mul(xfB.q, pc.localNormal);
					Vec2 planePoint = Utilities.Mul(xfB, pc.localPoint);

					Vec2 clipPoint = Utilities.Mul(xfA, pc.localPoints[index]);
					separation = Utilities.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
					point = clipPoint;

					// Ensure normal points from A to B
					normal = -normal;
				}
				break;
			}
		}

		public Vec2 normal;
		public Vec2 point;
		public float separation;
	};
}
