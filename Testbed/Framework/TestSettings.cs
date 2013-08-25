using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	/// Test settings. Some can be controlled in the GUI.
	class TestSettings {
		public TestSettings() {
			viewCenter = new Vec2(0.0f, 20.0f);
			hz = 60.0f;
			velocityIterations = 8;
			positionIterations = 3;
			drawShapes = true;
			drawJoints = true;
			drawAABBs = false;
			drawContactPoints = false;
			drawContactNormals = 0;
			drawContactImpulse = 0;
			drawFrictionImpulse = 0;
			drawCOMs = true;
			drawStats = false;
			drawProfile = false;
			enableWarmStarting = true;
			enableContinuous = true;
			enableSubStepping = false;
			enableSleep = true;
			pause = false;
			singleStep = false;
		}

		public Vec2 viewCenter;
		public float hz;
		public int velocityIterations;
		public int positionIterations;
		public bool drawShapes;
		public bool drawJoints;
		public bool drawAABBs;
		public bool drawContactPoints;
		public int drawContactNormals;
		public int drawContactImpulse;
		public int drawFrictionImpulse;
		public bool drawCOMs;
		public bool drawStats;
		public bool drawProfile;
		public bool enableWarmStarting;
		public bool enableContinuous;
		public bool enableSubStepping;
		public bool enableSleep;
		public bool pause;
		public bool singleStep;
	}
}
