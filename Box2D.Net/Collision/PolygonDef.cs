using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D
{
	/// <summary>
	/// Convex polygon. The vertices must be ordered so that the outside of
	/// the polygon is on the right side of the edges (looking along the edge
	/// from start to end).
	/// </summary>
	public class PolygonDef : FixtureDef
	{
		/// <summary>
		/// The number of polygon vertices.
		/// </summary>
		public int VertexCount
		{
			get
			{
				return polygon.GetVertexCount();
			}
		}

		/// <summary>
		/// The polygon vertices in local coordinates.
		/// </summary>
		public Vec2[] Vertices{
			get
			{
				return polygon.m_vertices;
			}
			set
			{
				polygon.m_vertices = value;
			}
		}

		public PolygonDef()
		{
			shape = new PolygonShape();
			Type = ShapeType.Polygon;
		}

		private PolygonShape polygon
		{
			get
			{
				return (PolygonShape)shape;
			}
		}

		/// <summary>
		/// Build vertices to represent an axis-aligned box.
		/// </summary>
		/// <param name="hx">The half-width</param>
		/// <param name="hy">The half-height.</param>
		public void SetAsBox(float hx, float hy)
		{
			polygon.SetAsBox(hx, hy);
		}


		/// <summary>
		/// Build vertices to represent an oriented box.
		/// </summary>
		/// <param name="hx">The half-width</param>
		/// <param name="hy">The half-height.</param>
		/// <param name="center">The center of the box in local coordinates.</param>
		/// <param name="angle">The rotation of the box in local coordinates.</param>
		public void SetAsBox(float hx, float hy, Vec2 center, float angle)
		{
			polygon.SetAsBox(hx, hy, center, angle);
		}
	}
}
