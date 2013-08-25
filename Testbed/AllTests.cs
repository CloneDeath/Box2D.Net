using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Testbed.Tests;

namespace Testbed {
	class AllTests {
		public static TestEntry[] GetTests() {
			return new TestEntry[]
			{
				new TestEntry("Apply Force", ApplyForce.Create),
				new TestEntry("Add Pair Stress Test", AddPair.Create),
				new TestEntry("Body Types", BodyTypes.Create),
				new TestEntry("Breakable", Breakable.Create),
				new TestEntry("Bridge", Bridge.Create),
				new TestEntry("Bullet Test", BulletTest.Create),
				new TestEntry("Cantilever", Cantilever.Create),
				new TestEntry("Car", Car.Create),
				new TestEntry("Chain", Chain.Create),
				new TestEntry("Character Collision", CharacterCollision.Create),
				new TestEntry("Collision Filtering", CollisionFiltering.Create),
				new TestEntry("Collision Processing", CollisionProcessing.Create),
				new TestEntry("Compound Shapes", CompoundShapes.Create),
				new TestEntry("Confined", Confined.Create),
				new TestEntry("Continuous Test", ContinuousTest.Create),
				new TestEntry("Convex Hull", ConvexHull.Create),
				new TestEntry("Conveyor Belt", ConveyorBelt.Create),
				new TestEntry("Distance Test", DistanceTest.Create),
				new TestEntry("Dominos", Dominos.Create),
				new TestEntry("Dump Shell", DumpShell.Create),
				new TestEntry("Dynamic Tree", DynamicTreeTest.Create),
				new TestEntry("Edge Shapes", EdgeShapes.Create),
				new TestEntry("Edge Test", EdgeTest.Create),
				new TestEntry("Gears", Gears.Create),
				new TestEntry("Mobile", Mobile.Create),
				new TestEntry("MobileBalanced", MobileBalanced.Create),
				new TestEntry("Motor Joint", MotorJointTest.Create),
				new TestEntry("One-Sided Platform", OneSidedPlatform.Create),
				new TestEntry("Pinball", Pinball.Create),
				new TestEntry("PolyCollision", PolyCollision.Create),
				new TestEntry("Polygon Shapes", PolyShapes.Create),
				new TestEntry("Prismatic", Prismatic.Create),
				new TestEntry("Pulleys", Pulleys.Create),
				new TestEntry("Pyramid", Pyramid.Create),
				new TestEntry("Ray-Cast", RayCast.Create),
				new TestEntry("Revolute", Revolute.Create),
				new TestEntry("Rope Test", RopeTest.Create),
				new TestEntry("RopeJoint", RopeJointTest.Create),
				new TestEntry("Sensor Test", SensorTest.Create),
				new TestEntry("Shape Editing", ShapeEditing.Create),
				new TestEntry("Slider Crank", SliderCrank.Create),
				new TestEntry("SphereStack", SphereStack.Create),
				new TestEntry("Theo Jansen's Walker", TheoJansen.Create),
				new TestEntry("Tiles", Tiles.Create),
				new TestEntry("Time of Impact", TimeOfImpact.Create),
				new TestEntry("Tumbler", Tumbler.Create),
				new TestEntry("Varying Friction", VaryingFriction.Create),
				new TestEntry("Varying Restitution", VaryingRestitution.Create),
				new TestEntry("Vertical Stack", VerticalStack.Create),
				new TestEntry("Web", Web.Create),
			};
		}
	}
}
