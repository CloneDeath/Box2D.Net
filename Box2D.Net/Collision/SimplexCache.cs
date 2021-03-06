﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Used to warm start Distance.
	/// Set count to zero on first call.
	public class SimplexCache
	{
		public SimplexCache(){
			indexA = new byte[3];
			indexB = new byte[3];
		}

		public float metric;		///< length or area
		public ushort count;
		public byte[] indexA;	///< vertices on shape A
		public byte[] indexB;	///< vertices on shape B
	};
}
