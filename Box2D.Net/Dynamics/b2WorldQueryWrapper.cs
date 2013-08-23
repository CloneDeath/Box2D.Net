using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2WorldQueryWrapper
	{
		public b2WorldQueryWrapper(object ignore) {
			broadPhase = new b2BroadPhase();
			callback = null;
		}

		bool QueryCallback(int proxyId)
		{
			b2FixtureProxy proxy = (b2FixtureProxy)broadPhase.GetUserData(proxyId);
			return callback.ReportFixture(proxy.fixture);
		}

		readonly b2BroadPhase broadPhase;
		b2QueryCallback callback;
	}
}
