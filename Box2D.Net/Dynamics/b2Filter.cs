﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This holds contact filtering data.
	public class Filter {
		public Filter() {
			categoryBits = 0x0001;
			maskBits = 0xFFFF;
			groupIndex = 0;
		}

		/// The collision category bits. Normally you would just set one bit.
		public ushort categoryBits;

		/// The collision mask bits. This states the categories that this
		/// shape would accept for collision.
		public ushort maskBits;

		/// Collision groups allow a certain group of objects to never collide (negative)
		/// or always collide (positive). Zero means no collision group. Non-zero group
		/// filtering always wins against the mask bits.
		public short groupIndex;
	};
}
