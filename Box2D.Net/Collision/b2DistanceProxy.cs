using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A distance proxy is used by the GJK algorithm.
	/// It encapsulates any shape.
	struct b2DistanceProxy
	{
		b2DistanceProxy() : m_vertices(null), m_count(0), m_radius(0.0f) {}

		/// Initialize the proxy using the given shape. The shape
		/// must remain in scope while the proxy is in use.
		void Set(const b2Shape* shape, int index);

		/// Get the supporting vertex index in the given direction.
		int GetSupport(const b2Vec2& d) const;

		/// Get the supporting vertex in the given direction.
		const b2Vec2& GetSupportVertex(const b2Vec2& d) const;

		/// Get the vertex count.
		int GetVertexCount() const;

		/// Get a vertex by index. Used by b2Distance.
		const b2Vec2& GetVertex(int index) const;

		b2Vec2 m_buffer[2];
		const b2Vec2* m_vertices;
		int m_count;
		float m_radius;
	};
}
