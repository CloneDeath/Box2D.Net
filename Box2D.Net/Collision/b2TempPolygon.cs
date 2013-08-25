using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// This holds polygon B expressed in frame A.
	class TempPolygon //was struct
	{
		public Vec2[] vertices = new Vec2[Settings._maxPolygonVertices];
		public Vec2[] normals = new Vec2[Settings._maxPolygonVertices];
		public int count;
	};
}
