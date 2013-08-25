using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A distance proxy is used by the GJK algorithm.
	/// It encapsulates any shape.
	public class DistanceProxy
	{
		public DistanceProxy(){
			m_vertices = new List<Vec2>();
			m_radius = 0.0f;
			m_buffer = new Vec2[2];
		}

		/// Initialize the proxy using the given shape. The shape
		/// must remain in scope while the proxy is in use.
		public void Set(Shape shape, int index){
			switch (shape.GetShapeType())
			{
			case ShapeType.Circle:
			    {
			        CircleShape circle = (CircleShape)shape;
					m_vertices = new List<Vec2>();
			        m_vertices.Add(circle.m_position);
			        m_radius = circle.m_radius;
			    }
			    break;

			case ShapeType.Polygon:
			    {
			        PolygonShape polygon = (PolygonShape)shape;
					m_vertices = new List<Vec2>();
			        m_vertices.AddRange(polygon.m_vertices);
			        m_radius = polygon.m_radius;
			    }
			    break;

			case ShapeType.Chain:
			    {
			        ChainShape chain = (ChainShape)shape;
			        Utilities.Assert(0 <= index && index < chain.m_count);

			        m_buffer[0] = chain.m_vertices[index];
			        if (index + 1 < chain.m_count)
			        {
			            m_buffer[1] = chain.m_vertices[index + 1];
			        }
			        else
			        {
			            m_buffer[1] = chain.m_vertices[0];
			        }

					m_vertices = new List<Vec2>();
			        m_vertices.AddRange(m_buffer);
			        m_radius = chain.m_radius;
			    }
			    break;

			case ShapeType.Edge:
			    {
			        EdgeShape edge = (EdgeShape)shape;
					m_vertices = new List<Vec2>();
					m_vertices.AddRange(new Vec2[]{edge.m_vertex1, edge.m_vertex2});
			        m_radius = edge.m_radius;
			    }
			    break;

			default:
			    Utilities.Assert(false);
				break;
			}
		}

		/// Get the supporting vertex index in the given direction.
		public int GetSupport(Vec2 d){
			int bestIndex = 0;
			float bestValue = Utilities.Dot(m_vertices[0], d);
			for (int i = 1; i < m_vertices.Count(); ++i) {
				float value = Utilities.Dot(m_vertices[i], d);
				if (value > bestValue) {
					bestIndex = i;
					bestValue = value;
				}
			}

			return bestIndex;
		}

		/// Get the supporting vertex in the given direction.
		public Vec2 GetSupportVertex(Vec2 d){
			throw new NotImplementedException();
			//int bestIndex = 0;
			//float bestValue = Utilities.Dot(m_vertices[0], d);
			//for (int i = 1; i < m_count; ++i)
			//{
			//    float value = Utilities.Dot(m_vertices[i], d);
			//    if (value > bestValue)
			//    {
			//        bestIndex = i;
			//        bestValue = value;
			//    }
			//}

			//return m_vertices[bestIndex];
		}

		/// Get the vertex count.
		public int GetVertexCount(){
			return m_vertices.Count();
		}

		/// Get a vertex by index. Used by Distance.
		public Vec2 GetVertex(int index) {
			Utilities.Assert(0 <= index && index < m_vertices.Count());
			return m_vertices[index];
		}

		public Vec2[] m_buffer;
		public List<Vec2> m_vertices;
		public float m_radius;
	};
}
