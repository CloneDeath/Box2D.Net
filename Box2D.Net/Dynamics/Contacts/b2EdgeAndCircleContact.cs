﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class b2EdgeAndCircleContact : b2Contact
	{
		public static b2Contact Create(b2Fixture fixtureA, int indexA, b2Fixture fixtureB, int indexB){
			return new b2EdgeAndCircleContact(fixtureA, fixtureB);
		}

		public static void Destroy(b2Contact contact){}

		public b2EdgeAndCircleContact(b2Fixture fixtureA, b2Fixture fixtureB) : base(fixtureA, 0, fixtureB, 0)
		{
			Utilities.Assert(m_fixtureA.GetShapeType() == ShapeType.Edge);
			Utilities.Assert(m_fixtureB.GetShapeType() == ShapeType.Circle);
		}

		public override void Evaluate(out b2Manifold manifold, b2Transform xfA, b2Transform xfB) {
			b2Collision.b2CollideEdgeAndCircle(out manifold,
										(b2EdgeShape)m_fixtureA.GetShape(), xfA,
										(b2CircleShape)m_fixtureB.GetShape(), xfB);
		}
	}
}
