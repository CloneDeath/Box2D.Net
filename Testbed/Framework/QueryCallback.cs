using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	class TestQueryCallback : QueryCallback
	{
		public TestQueryCallback(Vec2 point)
		{
			m_point = point;
			m_fixture = null;
		}

		public override bool ReportFixture(Fixture fixture)
		{
			throw new NotImplementedException();
			//Body body = fixture.GetBody();
			//if (body.GetType() == BodyType._dynamicBody)
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

		public Vec2 m_point;
		public Fixture m_fixture; //was pointer
	}
}
