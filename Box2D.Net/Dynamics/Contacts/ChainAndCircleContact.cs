using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class ChainAndCircleContact : Contact
	{
		public static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB){
			return new ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
		}

		public static void Destroy(Contact contact){}

		public ChainAndCircleContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB) : base(fixtureA, indexA, fixtureB, indexB)
		{
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Chain);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Circle);
		}

		public override void Evaluate(out Manifold manifold, Transform xfA, Transform xfB){
			ChainShape chain = (ChainShape)m_fixtureA.GetShape();
			EdgeShape edge;
			chain.GetChildEdge(out edge, m_indexA);
			Collision.CollideEdgeAndCircle(out manifold, edge, xfA,
									(CircleShape)m_fixtureB.GetShape(), xfB);
		}
	};
}
