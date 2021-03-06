﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This describes the motion of a body/shape for TOI computation.
	/// Shapes are defined with respect to the body origin, which may
	/// no coincide with the center of mass. However, to support dynamics
	/// we must interpolate the center of mass position.
	public struct Sweep
	{
		/// Get the interpolated transform at a specific time.
		/// @param beta is a factor in [0,1], where 0 indicates alpha0.
		public void GetTransform(out Transform xf, float beta) {
			xf = new Transform();
			xf.p = (1.0f - beta) * c0 + beta * c;
			float angle = (1.0f - beta) * a0 + beta * a;
			xf.q.Set(angle);

			// Shift to origin
			xf.p -= Utilities.Mul(xf.q, localCenter);
		}

		/// Advance the sweep forward, yielding a new initial state.
		/// @param alpha the new initial time.
		public void Advance(float alpha) {
			throw new NotImplementedException();
			//Utilities.Assert(alpha0 < 1.0f);
			//float beta = (alpha - alpha0) / (1.0f - alpha0);
			//c0 += beta * (c - c0);
			//a0 += beta * (a - a0);
			//alpha0 = alpha;
		}

		/// Normalize the angles.
		/// Normalize an angle in radians to be between -pi and pi
		public void Normalize() {
			float twoPi = 2.0f * (float)Math.PI;
			float d = twoPi * (float)Math.Floor(a0 / twoPi);
			a0 -= d;
			a -= d;
		}

		public Vec2 localCenter;	///< local center of mass position
		public Vec2 c0, c;		///< center world positions
		public float a0, a;		///< world angles

		/// Fraction of the current time step in the range [0,1]
		/// c0 and a0 are the positions at alpha0.
		public float alpha0;
	};
}
