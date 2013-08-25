using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class EdgeAndCircleContact : Contact
	{
		public static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB){
			return new EdgeAndCircleContact(fixtureA, fixtureB);
		}

		public static void Destroy(Contact contact){}

		public EdgeAndCircleContact(Fixture fixtureA, Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
		{
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Edge);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Circle);
		}

		public override void Evaluate(out Manifold manifold, Transform xfA, Transform xfB) {
			Collision.CollideEdgeAndCircle(out manifold,
										(EdgeShape)m_fixtureA.GetShape(), xfA,
										(CircleShape)m_fixtureB.GetShape(), xfB);
		}
	}
}
