using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This is a growable LIFO stack with an initial capacity of N.
	/// If the stack size exceeds the initial capacity, the heap is used
	/// to increase the size of the stack.
	class b2GrowableStack <T>
	{
		List<T> Stack;
		public b2GrowableStack(int Capacity)
		{
			Stack = new List<T>(Capacity);
		}

		public void Push(T element)
		{
			Stack.Add(element);
		}

		public T Pop()
		{
			T ret = Stack[0];
			Stack.RemoveAt(0);
			return ret;
		}

		public int GetCount()
		{
			return Stack.Count();
		}
	};
}
