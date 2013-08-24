using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Testbed.Framework {
	struct TestEntry
	{
		public delegate Test TestCreateFcn();

		public string name;
		public TestCreateFcn createFcn;
	}
}
