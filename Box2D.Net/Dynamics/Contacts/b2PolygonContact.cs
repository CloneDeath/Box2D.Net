using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class PolygonContact : Contact
	{
		public static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB){
			return new PolygonContact(fixtureA, fixtureB);
		}

		public static void Destroy(Contact contact){}

		public PolygonContact(Fixture fixtureA, Fixture fixtureB): base(fixtureA, 0, fixtureB, 0)
		{
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Polygon);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Polygon);
		}

		public override void Evaluate(out Manifold manifold, Transform xfA, Transform xfB) {
			Collision.CollidePolygons( out manifold,
								(PolygonShape)m_fixtureA.GetShape(), xfA,
								(PolygonShape)m_fixtureB.GetShape(), xfB);
		}
	}
}
