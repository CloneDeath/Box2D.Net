using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A node in the dynamic tree. The client does not interact with this directly.
	struct b2TreeNode
	{
		public const int b2_nullNode = -1;

		bool IsLeaf()
		{
			return child1 == b2_nullNode;
		}

		/// Enlarged AABB
		b2AABB aabb;

		object userData;

		//next and parent were in a union...
		int next;
		public int parent{
			get {
				return next;
			} 
			set {
				next = value;
			}
		}

		int child1;
		int child2;

		// leaf = 0, free node = -1
		int height;
	};
}
