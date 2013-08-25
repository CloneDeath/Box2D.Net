using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A node in the dynamic tree. The client does not interact with this directly.
	public class TreeNode
	{
		public const int _nullNode = -1;

		public bool IsLeaf()
		{
			return child1 == _nullNode;
		}

		/// Enlarged AABB
		public AABB aabb;

		public object userData;

		//next and parent were in a union...
		public int next;
		public int parent{
			get {
				return next;
			} 
			set {
				next = value;
			}
		}

		public int child1;
		public int child2;

		// leaf = 0, free node = -1
		public int height;
	};
}
