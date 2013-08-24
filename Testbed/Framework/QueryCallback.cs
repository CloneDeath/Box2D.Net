using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	class QueryCallback : b2QueryCallback
	{
		public QueryCallback(b2Vec2 point)
		{
			m_point = point;
			m_fixture = null;
		}

		public bool ReportFixture(b2Fixture fixture)
		{
			throw new NotImplementedException();
			//b2Body* body = fixture.GetBody();
			//if (body.GetType() == b2_dynamicBody)
			//{
			//    bool inside = fixture.TestPoint(m_point);
			//    if (inside)
			//    {
			//        m_fixture = fixture;

			//        // We are done, terminate the query.
			//        return false;
			//    }
			//}

			//// Continue the query.
			//return true;
		}

		public b2Vec2 m_point;
		public b2Fixture m_fixture; //was pointer
	}
}
