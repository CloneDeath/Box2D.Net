using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This holds contact filtering data.
	public class Filter {
		public Filter() {
			CategoryBits = 0x0001;
			MaskBits = 0xFFFF;
			GroupIndex = 0;
		}

		/// The collision category bits. Normally you would just set one bit.
		public ushort CategoryBits;

		/// The collision mask bits. This states the categories that this
		/// shape would accept for collision.
		public ushort MaskBits;

		/// Collision groups allow a certain group of objects to never collide (negative)
		/// or always collide (positive). Zero means no collision group. Non-zero group
		/// filtering always wins against the mask bits.
		public short GroupIndex;
	};
}
