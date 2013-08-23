using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D.Dynamics {
	struct b2WorldRayCastWrapper
	{
		public b2WorldRayCastWrapper(object ignore) {
			broadPhase = new b2BroadPhase();
			callback = null;
		}

		float RayCastCallback(b2RayCastInput input, int proxyId)
		{
			
			throw new NotImplementedException();
			//void* userData = broadPhase.GetUserData(proxyId);
			//b2FixtureProxy* proxy = (b2FixtureProxy*)userData;
			//b2Fixture* fixture = proxy.fixture;
			//int index = proxy.childIndex;
			//b2RayCastOutput output;
			//bool hit = fixture.RayCast(&output, input, index);

			//if (hit)
			//{
			//    float fraction = output.fraction;
			//    b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			//    return callback.ReportFixture(fixture, point, output.normal, fraction);
			//}

			//return input.maxFraction;
		}

		public readonly b2BroadPhase broadPhase;
		b2RayCastCallback callback;
	}
}
