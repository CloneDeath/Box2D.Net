using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public class Settings {
		/// @file
		/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
		///

		// Collision

		/// The maximum number of contact points between two convex shapes. Do
		/// not change this value.
		public const int _maxManifoldPoints = 2;

		/// The maximum number of vertices on a convex polygon. You cannot increase
		/// this too much because BlockAllocator has a maximum object size.
		public const int _maxPolygonVertices = 8;
		

		/// This is used to fatten AABBs in the dynamic tree. This allows proxies
		/// to move by a small amount without triggering a tree adjustment.
		/// This is in meters.
		public const float _aabbExtension = 0.1f;

		/// This is used to fatten AABBs in the dynamic tree. This is used to predict
		/// the future position based on the current displacement.
		/// This is a dimensionless multiplier.
		public const float _aabbMultiplier	=	2.0f;

		/// A small length used as a collision and constraint tolerance. Usually it is
		/// chosen to be numerically significant, but visually insignificant.
		public const float _linearSlop	=		0.005f;

		/// A small angle used as a collision and constraint tolerance. Usually it is
		/// chosen to be numerically significant, but visually insignificant.
		public const float _angularSlop		=	(float)(2.0f / 180.0f * (float)Math.PI);

		/// The radius of the polygon/edge shape skin. This should not be modified. Making
		/// this smaller means polygons will have an insufficient buffer for continuous collision.
		/// Making it larger may create artifacts for vertex collision.
		public const float _polygonRadius		=(2.0f *Settings._linearSlop);

		/// Maximum number of sub-steps per contact in continuous physics simulation.
		public const int _maxSubSteps		=	8;


		// Dynamics

		/// Maximum number of contacts to be handled to solve a TOI impact.
		public const int _maxTOIContacts	=		32;

		/// A velocity threshold for elastic collisions. Any collision with a relative linear
		/// velocity below this threshold will be treated as inelastic.
		public const float _velocityThreshold		=1.0f;

		/// The maximum linear position correction used when solving constraints. This helps to
		/// prevent overshoot.
		public const float _maxLinearCorrection	=	0.2f;

		/// The maximum angular position correction used when solving constraints. This helps to
		/// prevent overshoot.
		public const float _maxAngularCorrection	=	(float)(8.0f / 180.0f * (float)Math.PI);

		/// The maximum linear velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		public const float _maxTranslation	=		2.0f;
		public const float _maxTranslationSquared	=(_maxTranslation * _maxTranslation);

		/// The maximum angular velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		public const float _maxRotation		=		(float)(0.5f * (float)Math.PI);
		public const float _maxRotationSquared	=	(_maxRotation * _maxRotation);

		/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
		/// that overlap is removed in one time step. However using values close to 1 often lead
		/// to overshoot.
		public const float _baumgarte		=		0.2f;
		public const float _toiBaugarte	=			0.75f;


		// Sleep

		/// The time that a body must be still before it will go to sleep.
		public const float _timeToSleep		=		0.5f;

		/// A body cannot sleep if its linear velocity is above this tolerance.
		public const float _linearSleepTolerance	=	0.01f;

		/// A body cannot sleep if its angular velocity is above this tolerance.
		public const float _angularSleepTolerance	= (float)(2.0f / 180.0f * (float)Math.PI);

		// Memory Allocation

		/// Logging function.
		public static void Log(string message, params object[] etc ){
			Console.WriteLine(String.Format(message, etc));
		}

		/// Version numbering scheme.
		/// See http://en.wikipedia.org/wiki/Software_versioning
		public struct Version
		{
			internal Version(int a, int b, int c) {
				major = a;
				minor = b;
				revision = c;
			}
			public int major;		///< significant changes
			public int minor;		///< incremental changes
			public int revision;		///< bug fixes
		};

		/// Current version.
		public static Version _version = new Version(1, 0, 0);
	}
}
