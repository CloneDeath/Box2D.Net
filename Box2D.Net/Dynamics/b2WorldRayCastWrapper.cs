using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D.Dynamics {
	struct WorldRayCastWrapper
	{
		public WorldRayCastWrapper(object ignore) {
			broadPhase = new BroadPhase();
			callback = null;
		}

		float RayCastCallback(RayCastInput input, int proxyId)
		{
			
			throw new NotImplementedException();
			//void* userData = broadPhase.GetUserData(proxyId);
			//FixtureProxy* proxy = (FixtureProxy*)userData;
			//Fixture* fixture = proxy.fixture;
			//int index = proxy.childIndex;
			//RayCastOutput output;
			//bool hit = fixture.RayCast(out output, input, index);

			//if (hit)
			//{
			//    float fraction = output.fraction;
			//    Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			//    return callback.ReportFixture(fixture, point, output.normal, fraction);
			//}

			//return input.maxFraction;
		}

		public readonly BroadPhase broadPhase;
		RayCastCallback callback;
	}
}
