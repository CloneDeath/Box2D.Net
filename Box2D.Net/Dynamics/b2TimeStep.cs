﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Profiling data. Times are in milliseconds.
	struct b2Profile {
		public float step;
		public float collide;
		public float solve;
		public float solveInit;
		public float solveVelocity;
		public float solvePosition;
		public float broadphase;
		public float solveTOI;
	}

	/// This is an internal structure.
	struct b2TimeStep {
		public float dt;			// time step
		public float inv_dt;		// inverse time step (0 if dt == 0).
		public float dtRatio;	// dt * inv_dt0
		public int velocityIterations;
		public int positionIterations;
		public bool warmStarting;
	}

	/// This is an internal structure.
	struct b2Position {
		public b2Vec2 c;
		public float a;
	}

	/// This is an internal structure.
	struct b2Velocity {
		public b2Vec2 v;
		public float w;
	}

	/// Solver Data
	struct b2SolverData {
		public b2TimeStep step;
		public List<b2Position> positions; //was pointer
		public List<b2Velocity> velocities; //was pointer
	}
}
