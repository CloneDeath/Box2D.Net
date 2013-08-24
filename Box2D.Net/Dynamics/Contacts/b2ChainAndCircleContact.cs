using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class b2ChainAndCircleContact : b2Contact
	{
		public static b2Contact Create(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB){
			return new b2ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
		}

		public static void Destroy(b2Contact contact){}

		public b2ChainAndCircleContact(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB) : base(fixtureA, indexA, fixtureB, indexB)
		{
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Chain);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Circle);
		}

		public override void Evaluate(out b2Manifold manifold, b2Transform xfA, b2Transform xfB){
			b2ChainShape chain = (b2ChainShape)m_fixtureA.GetShape();
			b2EdgeShape edge;
			chain.GetChildEdge(out edge, m_indexA);
			b2Collision.b2CollideEdgeAndCircle(out manifold, edge, xfA,
									(b2CircleShape)m_fixtureB.GetShape(), xfB);
		}
	};
}
