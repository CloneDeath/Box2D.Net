using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class CircleContact : Contact
	{
		public static Contact Create(Fixture fixtureA, int indexA,
									   Fixture fixtureB, int indexB){
			return new CircleContact(fixtureA, fixtureB);
		}

		public static void Destroy(Contact contact){
			
		}

		public CircleContact(Fixture fixtureA, Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0){
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Circle);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Circle);
		}

		public override void Evaluate(out Manifold manifold, Transform xfA, Transform xfB){
			Collision.CollideCircles(out manifold,
							(CircleShape)m_fixtureA.GetShape(), xfA,
							(CircleShape)m_fixtureB.GetShape(), xfB);
		}
	};
}
