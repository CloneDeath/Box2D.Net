﻿using System;
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

		/// Use this to store application specific fixture data.
		public object UserData { get; set; }

		/// The friction coefficient, usually in the range [0,1].
		public float friction;

		/// The restitution (elasticity) usually in the range [0,1].
		public float restitution;

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
			shape = null;
			UserData = null;
			friction = 0.2f;
			restitution = 0.0f;
			Density = 0.0f;
			IsSensor = false;
			Filter = new Filter();
		}
	}
}
