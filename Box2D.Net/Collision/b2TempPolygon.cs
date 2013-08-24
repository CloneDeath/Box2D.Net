using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// This holds polygon B expressed in frame A.
	class b2TempPolygon //was struct
	{
		public b2Vec2[] vertices = new b2Vec2[b2Settings.b2_maxPolygonVertices];
		public b2Vec2[] normals = new b2Vec2[b2Settings.b2_maxPolygonVertices];
		public int count;
	};
}
