using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A transform contains translation and rotation. It is used to represent
	/// the position and orientation of rigid frames.
	public struct Transform {
		public Vec2 p;
		public Rot q;

		/// The default constructor does nothing.
		public Transform(object ignore) {
			p = new Vec2();
			q = new Rot();
		}

		/// Initialize using a position vector and a rotation.
		public Transform(Vec2 position, Rot rotation){
			this.p = position;
			this.q = rotation;
		}

		/// Set this to the identity transform.
		public void SetIdentity()
		{
			p.SetZero();
			q.SetIdentity();
		}

		/// Set this based on the position and angle.
		public void Set(Vec2 position, float angle)
		{
			p = position;
			q.Set(angle);
		}
	}
}
