using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A transform contains translation and rotation. It is used to represent
	/// the position and orientation of rigid frames.
	public struct b2Transform {
		public b2Vec2 p;
		public b2Rot q;

		/// The default constructor does nothing.
		public b2Transform() {
			p = new b2Vec2();
			q = new b2Rot();
		}

		/// Initialize using a position vector and a rotation.
		public b2Transform(b2Vec2 position, b2Rot rotation){
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
		public void Set(b2Vec2 position, float angle)
		{
			p = position;
			q.Set(angle);
		}
	}
}
