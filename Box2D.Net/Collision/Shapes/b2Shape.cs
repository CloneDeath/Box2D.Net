using System;
using System.Collections.Generic;
using System.Text;

namespace Box2D {
	public enum ShapeType
	{
		Circle = 0,
		Edge = 1,
		Polygon = 2,
		Chain = 3,
		Count = 4
	};
	
	/// A shape is used for collision detection. You can create a shape however you like.
	/// Shapes used for simulation in b2World are created automatically when a b2Fixture
	/// is created. Shapes may encapsulate a one or more child shapes.
	public abstract class b2Shape {
		public ShapeType m_type;
		public float m_radius;

		public ~b2Shape() {}

		/// Clone the concrete shape using the provided allocator.
		public abstract b2Shape Clone();

		/// Get the type of this shape. You can use this to down cast to the concrete shape.
		/// @return the shape type.
		public ShapeType GetShapeType() {
			return m_type;
		}

		/// Get the number of child primitives.
		public abstract int GetChildCount();

		/// Test a point for containment in this shape. This only works for convex shapes.
		/// @param xf the shape world transform.
		/// @param p a point in world coordinates.
		public abstract bool TestPoint(b2Transform xf, b2Vec2 p);

		/// Cast a ray against a child shape.
		/// @param output the ray-cast results.
		/// @param input the ray-cast input parameters.
		/// @param transform the transform to be applied to the shape.
		/// @param childIndex the child shape index
		public abstract bool RayCast(out b2RayCastOutput output, b2RayCastInput input,
							b2Transform transform, int childIndex);

		/// Given a transform, compute the associated axis aligned bounding box for a child shape.
		/// @param aabb returns the axis aligned box.
		/// @param xf the world transform of the shape.
		/// @param childIndex the child shape
		public abstract void ComputeAABB(out b2AABB aabb, b2Transform xf, int childIndex);

		/// Compute the mass properties of this shape using its dimensions and density.
		/// The inertia tensor is computed about the local origin.
		/// @param massData returns the mass data for this shape.
		/// @param density the density in kilograms per meter squared.
		public abstract void ComputeMass(out b2MassData massData, float density);
	}
}
