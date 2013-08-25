using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Rotation
	public struct Rot {
		/// Sine and cosine
		public float s, c;

		public Rot(object ignore) {
			s = 0;
			c = 1;
		}

		/// Initialize from an angle in radians
		public Rot(float angle)
		{
			/// TODO_ERIN optimize
			s = (float)Math.Sin(angle);
			c = (float)Math.Cos(angle);
		}

		/// Set using an angle in radians.
		public void Set(float angle)
		{
			/// TODO_ERIN optimize
			s = (float)Math.Sin(angle);
			c = (float)Math.Cos(angle);
		}

		/// Set to the identity rotation
		public void SetIdentity()
		{
			s = 0.0f;
			c = 1.0f;
		}

		/// Get the angle in radians
		public float GetAngle()
		{
			return (float)Math.Atan2(s, c);
		}

		/// Get the x-axis
		public Vec2 GetXAxis()
		{
			return new Vec2(c, s);
		}

		/// Get the u-axis
		public Vec2 GetYAxis()
		{
			return new Vec2(-s, c);
		}
	}
}
