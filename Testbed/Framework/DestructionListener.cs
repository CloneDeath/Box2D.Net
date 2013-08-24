using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	// This is called when a joint in the world is implicitly destroyed
	// because an attached body is destroyed. This gives us a chance to
	// nullify the mouse joint.
	class DestructionListener : b2DestructionListener
	{
		public override void SayGoodbye(b2Fixture fixture) {  }
		public override void SayGoodbye(b2Joint joint) {
			if (test.m_mouseJoint == joint) {
				test.m_mouseJoint = null;
			} else {
				test.JointDestroyed(joint);
			}
		}

		public Test test;
	}
}
