using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D
{
	/// <summary>
	/// This structure is used to build a chain of edges.
	/// </summary>
	public class EdgeDef : FixtureDef
	{
		public EdgeDef()
		{
			Type = ShapeType.Edge;
			shape = new EdgeShape();
		}

		private EdgeShape edge
		{
			get
			{
				return (EdgeShape)shape;
			}
		}

		/// <summary>
		/// The start vertex.
		/// </summary>
		public Vec2 Vertex1
		{
			get
			{
				return edge.m_vertex1;
			}
			set
			{
				edge.m_vertex1 = value;
			}
		}

		/// <summary>
		/// The end vertex.
		/// </summary>
		public Vec2 Vertex2
		{
			get
			{
				return edge.m_vertex2;
			}
			set
			{
				edge.m_vertex2 = value;
			}
		}
	}
}
