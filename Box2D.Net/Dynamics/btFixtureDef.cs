using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A fixture definition is used to create a fixture. This class defines an
	/// abstract fixture definition. You can reuse fixture definitions safely.
	abstract class b2FixtureDef
	{
		/// The shape, this must be set. The shape will be cloned, so you
		/// can create the shape on the stack.
		b2Shape shape; //was const*

		/// Use this to store application specific fixture data.
		object userData;

		/// The friction coefficient, usually in the range [0,1].
		float friction;

		/// The restitution (elasticity) usually in the range [0,1].
		float restitution;

		/// The density, usually in kg/m^2.
		float density;

		/// A sensor shape collects contact information but never generates a collision
		/// response.
		bool isSensor;

		/// Contact filtering data.
		b2Filter filter;

		/// The constructor sets the default fixture definition values.
		public b2FixtureDef()
		{
			shape = null;
			userData = null;
			friction = 0.2f;
			restitution = 0.0f;
			density = 0.0f;
			isSensor = false;
		}

		
	}
}
