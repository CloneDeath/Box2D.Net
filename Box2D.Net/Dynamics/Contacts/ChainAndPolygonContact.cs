using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class ChainAndPolygonContact : Contact
	{
		public static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB){
			return new ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB);
		}

		public static void Destroy(Contact contact){}

		public ChainAndPolygonContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB) : base(fixtureA, indexA, fixtureB, indexB)
		{
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Chain);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Polygon);
		}


		public override void Evaluate(out Manifold manifold, Transform xfA, Transform xfB) {
			ChainShape chain = (ChainShape)m_fixtureA.GetShape();
			EdgeShape edge;
			chain.GetChildEdge(out edge, m_indexA);
			Collision.CollideEdgeAndPolygon(out manifold, edge, xfA,
										(PolygonShape)m_fixtureB.GetShape(), xfB);
		}
	}
}
