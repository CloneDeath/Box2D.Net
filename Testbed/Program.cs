using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GLImp;
using Box2D;
using Testbed.Framework;

namespace Testbed {
	class Program {
		extern TestEntry[] g_testEntries;
		const int k_maxContactPoints = 2048;



		public static void Main(string[] args) {
			

			GraphicsManager.Start();
		}
	}
}
