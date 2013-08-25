using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A chain shape is a free form sequence of line segments.
	/// The chain has two-sided collision, so you can use inside and outside collision.
	/// Therefore, you may use any winding order.
	/// Since there may be many vertices, they are allocated using Alloc.
	/// Connectivity information is used to create smooth collisions.
	/// WARNING: The chain will not collide properly if there are self-intersections.
	public class ChainShape : Shape {
		public ChainShape(){
			m_type = ShapeType.Chain;
			m_radius = Settings._polygonRadius;
			m_vertices = new List<Vec2>();
			m_count = 0;
			m_hasPrevVertex = false;
			m_hasNextVertex = false;
		}

		/// The destructor frees the vertices using Free.
		~ChainShape(){
			m_vertices = null;
			m_count = 0;
		}

		/// Create a loop. This automatically adjusts connectivity.
		/// @param vertices an array of vertices, these are copied
		/// @param count the vertex count
		public void CreateLoop(Vec2[] vertices, int count){
			throw new NotImplementedException();
			//Utilities.Assert(m_vertices == null && m_count == 0);
			//Utilities.Assert(count >= 3);
			//for (int i = 1; i < count; ++i)
			//{
			//    Vec2 v1 = vertices[i-1];
			//    Vec2 v2 = vertices[i];
			//    // If the code crashes here, it means your vertices are too close together.
			//    Utilities.Assert(DistanceSquared(v1, v2) >Settings._linearSlop *Settings._linearSlop);
			//}

			//m_count = count + 1;
			//m_vertices = (Vec2*)Alloc(m_count * sizeof(Vec2));
			//memcpy(m_vertices, vertices, count * sizeof(Vec2));
			//m_vertices[count] = m_vertices[0];
			//m_prevVertex = m_vertices[m_count - 2];
			//m_nextVertex = m_vertices[1];
			//m_hasPrevVertex = true;
			//m_hasNextVertex = true;
		}

		/// Create a chain with isolated end vertices.
		/// @param vertices an array of vertices, these are copied
		/// @param count the vertex count
		public void CreateChain(Vec2[] vertices, int count){
			throw new NotImplementedException();
			//Utilities.Assert(m_vertices == null && m_count == 0);
			//Utilities.Assert(count >= 2);
			//for (int i = 1; i < count; ++i)
			//{
			//    Vec2 v1 = vertices[i-1];
			//    Vec2 v2 = vertices[i];
			//    // If the code crashes here, it means your vertices are too close together.
			//    Utilities.Assert(DistanceSquared(v1, v2) >Settings._linearSlop *Settings._linearSlop);
			//}

			//m_count = count;
			//m_vertices = (Vec2*)Alloc(count * sizeof(Vec2));
			//memcpy(m_vertices, vertices, m_count * sizeof(Vec2));

			//m_hasPrevVertex = false;
			//m_hasNextVertex = false;
		}

		/// Establish connectivity to a vertex that precedes the first vertex.
		/// Don't call this for loops.
		public void SetPrevVertex(Vec2 prevVertex){
			m_prevVertex = prevVertex;
			m_hasPrevVertex = true;
		}

		/// Establish connectivity to a vertex that follows the last vertex.
		/// Don't call this for loops.
		public void SetNextVertex(Vec2 nextVertex){
			m_nextVertex = nextVertex;
			m_hasNextVertex = true;
		}

		/// Implement Shape. Vertices are cloned using Alloc.
		public override Shape Clone() {
			throw new NotImplementedException();
			//void* mem = allocator.Allocate(sizeof(ChainShape));
			//ChainShape* clone = new (mem) ChainShape;
			//clone.CreateChain(m_vertices, m_count);
			//clone.m_prevVertex = m_prevVertex;
			//clone.m_nextVertex = m_nextVertex;
			//clone.m_hasPrevVertex = m_hasPrevVertex;
			//clone.m_hasNextVertex = m_hasNextVertex;
			//return clone;
		}

		/// @see Shape::GetChildCount
		public override int GetChildCount() {
			// edge count = vertex count - 1
			return m_count - 1;
		}

		/// Get a child edge.
		public void GetChildEdge(out EdgeShape edge, int index){
			throw new NotImplementedException();
			//Utilities.Assert(0 <= index && index < m_count - 1);
			//edge.m_type = ShapeType.edge;
			//edge.m_radius = m_radius;

			//edge.m_vertex1 = m_vertices[index + 0];
			//edge.m_vertex2 = m_vertices[index + 1];

			//if (index > 0)
			//{
			//    edge.m_vertex0 = m_vertices[index - 1];
			//    edge.m_hasVertex0 = true;
			//}
			//else
			//{
			//    edge.m_vertex0 = m_prevVertex;
			//    edge.m_hasVertex0 = m_hasPrevVertex;
			//}

			//if (index < m_count - 2)
			//{
			//    edge.m_vertex3 = m_vertices[index + 2];
			//    edge.m_hasVertex3 = true;
			//}
			//else
			//{
			//    edge.m_vertex3 = m_nextVertex;
			//    edge.m_hasVertex3 = m_hasNextVertex;
			//}
		}

		/// This always return false.
		/// @see Shape::TestPoint
		public override bool TestPoint(Transform transform, Vec2 p) {
			return false;
		}

		/// Implement Shape.
		public override bool RayCast(out RayCastOutput output, RayCastInput input,
						Transform transform, int childIndex){
			throw new NotImplementedException();
			//Utilities.Assert(childIndex < m_count);

			//EdgeShape edgeShape;

			//int i1 = childIndex;
			//int i2 = childIndex + 1;
			//if (i2 == m_count)
			//{
			//    i2 = 0;
			//}

			//edgeShape.m_vertex1 = m_vertices[i1];
			//edgeShape.m_vertex2 = m_vertices[i2];

			//return edgeShape.RayCast(output, input, xf, 0);
		}

		/// @see Shape::ComputeAABB
		public override void ComputeAABB(out AABB aabb, Transform transform, int childIndex) {
			throw new NotImplementedException();
			//Utilities.Assert(childIndex < m_count);

			//int i1 = childIndex;
			//int i2 = childIndex + 1;
			//if (i2 == m_count)
			//{
			//    i2 = 0;
			//}

			//Vec2 v1 = Utilities.Mul(xf, m_vertices[i1]);
			//Vec2 v2 = Utilities.Mul(xf, m_vertices[i2]);

			//aabb.lowerBound = Math.Min(v1, v2);
			//aabb.upperBound = Math.Max(v1, v2);
		}

		/// Chains have zero mass.
		/// @see Shape::ComputeMass
		public override void ComputeMass(out MassData massData, float Density){
			massData = new MassData();
			massData.mass = 0.0f;
			massData.center.SetZero();
			massData.I = 0.0f;
		}

		/// The vertices. Owned by this class.
		public List<Vec2> m_vertices;

		/// The vertex count.
		public int m_count;

		public Vec2 m_prevVertex, m_nextVertex;
		public bool m_hasPrevVertex, m_hasNextVertex;
	}
}
