using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct WorldQueryWrapper
	{
		public WorldQueryWrapper(object ignore) {
			broadPhase = new BroadPhase();
			callback = null;
		}

		bool QueryCallback(int proxyId)
		{
			FixtureProxy proxy = (FixtureProxy)broadPhase.GetUserData(proxyId);
			return callback.ReportFixture(proxy.fixture);
		}

		readonly BroadPhase broadPhase;
		QueryCallback callback;
	}
}
