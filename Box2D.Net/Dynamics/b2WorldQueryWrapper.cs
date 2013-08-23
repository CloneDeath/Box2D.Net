using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2WorldQueryWrapper
	{
		bool QueryCallback(int proxyId)
		{
			b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
			return callback->ReportFixture(proxy->fixture);
		}

		const b2BroadPhase* broadPhase;
		b2QueryCallback* callback;
	}
}
