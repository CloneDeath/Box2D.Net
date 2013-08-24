using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	/// Test settings. Some can be controlled in the GUI.
	struct Settings {
		public Settings(object ignore) {
			viewCenter = new b2Vec2(0.0f, 20.0f);
			hz = 60.0f;
			velocityIterations = 8;
			positionIterations = 3;
			drawShapes = 1;
			drawJoints = 1;
			drawAABBs = 0;
			drawContactPoints = 0;
			drawContactNormals = 0;
			drawContactImpulse = 0;
			drawFrictionImpulse = 0;
			drawCOMs = 0;
			drawStats = 0;
			drawProfile = 0;
			enableWarmStarting = 1;
			enableContinuous = 1;
			enableSubStepping = 0;
			enableSleep = 1;
			pause = 0;
			singleStep = 0;
		}

		b2Vec2 viewCenter;
		float hz;
		int velocityIterations;
		int positionIterations;
		int drawShapes;
		int drawJoints;
		int drawAABBs;
		int drawContactPoints;
		int drawContactNormals;
		int drawContactImpulse;
		int drawFrictionImpulse;
		int drawCOMs;
		int drawStats;
		int drawProfile;
		int enableWarmStarting;
		int enableContinuous;
		int enableSubStepping;
		int enableSleep;
		int pause;
		int singleStep;
	}
}
