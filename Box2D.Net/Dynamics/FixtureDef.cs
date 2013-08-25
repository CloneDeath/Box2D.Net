using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A fixture definition is used to create a fixture. This class defines an
	/// abstract fixture definition. You can reuse fixture definitions safely.
	public class FixtureDef
	{
		/// The shape, this must be set. The shape will be cloned, so you
		/// can create the shape on the stack.
		public Shape shape; //was const*

		public ShapeType Type;

		/// Use this to store application specific fixture data.
		public object UserData { get; set; }

		/// The friction coefficient, usually in the range [0,1].
		public float Friction;

		/// The restitution (elasticity) usually in the range [0,1].
		public float Restitution;

		/// The Density, usually in kg/m^2.
		public float Density { get; set; }

		/// A sensor shape collects contact information but never generates a collision
		/// response.
		public bool IsSensor { get; set; }

		/// Contact filtering data.
		public Filter Filter { get; set; }

		/// The constructor sets the default fixture definition values.
		public FixtureDef()
		{
			Type = ShapeType.Unknown;
			shape = null;
			UserData = null;
			Friction = 0.2f;
			Restitution = 0.0f;
			Density = 0.0f;
			IsSensor = false;
			Filter = new Filter();
		}
	}
}
