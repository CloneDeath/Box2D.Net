using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A shape is used for collision detection. You can create a shape however you like.
	/// Shapes used for simulation in b2World are created automatically when a b2Fixture
	/// is created. Shapes may encapsulate a one or more child shapes.
	class b2Shape {
		public enum Type
		{
			e_circle = 0,
			e_edge = 1,
			e_polygon = 2,
			e_chain = 3,
			e_typeCount = 4
		};

		public virtual ~b2Shape() {}

		/// Clone the concrete shape using the provided allocator.
		public virtual b2Shape* Clone(b2BlockAllocator* allocator) const = 0;

		/// Get the type of this shape. You can use this to down cast to the concrete shape.
		/// @return the shape type.
		public Type GetType() const{
			return m_type;
		}

		/// Get the number of child primitives.
		public virtual int GetChildCount() const = 0;

		/// Test a point for containment in this shape. This only works for convex shapes.
		/// @param xf the shape world transform.
		/// @param p a point in world coordinates.
		public virtual bool TestPoint(const b2Transform& xf, const b2Vec2& p) const = 0;

		/// Cast a ray against a child shape.
		/// @param output the ray-cast results.
		/// @param input the ray-cast input parameters.
		/// @param transform the transform to be applied to the shape.
		/// @param childIndex the child shape index
		public virtual bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& transform, int childIndex) const = 0;

		/// Given a transform, compute the associated axis aligned bounding box for a child shape.
		/// @param aabb returns the axis aligned box.
		/// @param xf the world transform of the shape.
		/// @param childIndex the child shape
		public virtual void ComputeAABB(b2AABB* aabb, const b2Transform& xf, int childIndex) const = 0;

		/// Compute the mass properties of this shape using its dimensions and density.
		/// The inertia tensor is computed about the local origin.
		/// @param massData returns the mass data for this shape.
		/// @param density the density in kilograms per meter squared.
		public virtual void ComputeMass(b2MassData* massData, float density) const = 0;

		public Type m_type;
		public float m_radius;
	}
}
