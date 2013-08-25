using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;

namespace Box2D {
	public class b2Timer {
		Stopwatch timer;

		/// Constructor
		public b2Timer(){
			timer = new Stopwatch();
			timer.Start();
		}

		/// Reset the timer.
		public void Reset() {
			timer.Restart();
		}

		/// Get the time since construction or the last reset.
		public float GetMilliseconds() {
			return timer.ElapsedMilliseconds;
		}
	}
}
