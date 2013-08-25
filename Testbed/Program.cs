using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using GLImp;
using Box2D;
using Testbed.Framework;
using Gwen.Control;
using OpenTK.Input;
using Testbed.Tests;

namespace Testbed {
	class Program {
		public static TestEntry[] g_testEntries;
		public const int k_maxContactPoints = 2048;

		static int testIndex = 0;
		static int testSelection = 0;
		static int testCount = 0;
		static TestEntry entry; //pointer
		static Test test; //pointer
		static Settings settings = new Settings();
		static int width = 800;
		static int height = 600;
		static int framePeriod = 16;
		static int mainWindow;
		static float settingsHz = 60.0f;
		static float viewZoom = 12.0f;
		static int tx, ty, tw, th;
		static bool rMouseDown;
		static b2Vec2 lastp;

		static Camera2D camera;
		static ListBox testList;
		public static void Main(string[] args) {
			testCount = 0;
			g_testEntries = new TestEntry[]
			{
				new TestEntry("Continuous Test", ContinuousTest.Create),
				new TestEntry("Time of Impact", TimeOfImpact.Create),
				new TestEntry("Motor Joint", MotorJoint.Create),
				new TestEntry("One-Sided Platform", OneSidedPlatform.Create),
				new TestEntry("Dump Shell", DumpShell.Create),
				new TestEntry("Mobile", Mobile.Create),
				new TestEntry("MobileBalanced", MobileBalanced.Create),
				new TestEntry("Ray-Cast", RayCast.Create),
				new TestEntry("Conveyor Belt", ConveyorBelt.Create),
				new TestEntry("Gears", Gears.Create),
				new TestEntry("Convex Hull", ConvexHull.Create),
				new TestEntry("Varying Restitution", VaryingRestitution.Create),
				new TestEntry("Tumbler", Tumbler.Create),
				new TestEntry("Tiles", Tiles.Create),
				new TestEntry("Cantilever", Cantilever.Create),
				new TestEntry("Character Collision", CharacterCollision.Create),
				new TestEntry("Edge Test", EdgeTest.Create),
				new TestEntry("Body Types", BodyTypes.Create),
				new TestEntry("Shape Editing", ShapeEditing.Create),
				new TestEntry("Car", Car.Create),
				new TestEntry("Apply Force", ApplyForce.Create),
				new TestEntry("Prismatic", Prismatic.Create),
				new TestEntry("Vertical Stack", VerticalStack.Create),
				new TestEntry("SphereStack", SphereStack.Create),
				new TestEntry("Revolute", Revolute.Create),
				new TestEntry("Pulleys", Pulleys.Create),
				new TestEntry("Polygon Shapes", PolyShapes.Create),
				new TestEntry("Web", Web.Create),
				new TestEntry("RopeJoint", RopeJoint.Create),
				new TestEntry("Pinball", Pinball.Create),
				new TestEntry("Bullet Test", BulletTest.Create),
				new TestEntry("Confined", Confined.Create),
				new TestEntry("Pyramid", Pyramid.Create),
				new TestEntry("Theo Jansen's Walker", TheoJansen.Create),
				new TestEntry("Edge Shapes", EdgeShapes.Create),
				new TestEntry("PolyCollision", PolyCollision.Create),
				new TestEntry("Bridge", Bridge.Create),
				new TestEntry("Breakable", Breakable.Create),
				new TestEntry("Chain", Chain.Create),
				new TestEntry("Collision Filtering", CollisionFiltering.Create),
				new TestEntry("Collision Processing", CollisionProcessing.Create),
				new TestEntry("Compound Shapes", CompoundShapes.Create),
				new TestEntry("Distance Test", DistanceTest.Create),
				new TestEntry("Dominos", Dominos.Create),
				new TestEntry("Dynamic Tree", DynamicTreeTest.Create),
				new TestEntry("Sensor Test", SensorTest.Create),
				new TestEntry("Slider Crank", SliderCrank.Create),
				new TestEntry("Varying Friction", VaryingFriction.Create),
				new TestEntry("Add Pair Stress Test", AddPair.Create),
			};

			while (testCount < g_testEntries.Count())
			{
				++testCount;
			}

			testIndex = Math.Max(0, Math.Min(testIndex, testCount - 1));
			testSelection = testIndex;

			entry = g_testEntries[testIndex];
			test = entry.createFcn();

			GraphicsManager.SetResolution(width, height);
			string title = String.Format("Box2D Version {0}.{1}.{2}", b2Settings.b2_version.major, b2Settings.b2_version.minor, b2Settings.b2_version.revision);
			GraphicsManager.SetTitle(title);

			camera = new Camera2D();
			camera.OnRender += SimulationLoop;

			camera.SetZoom(12);
			camera.CenterOnTarget(true);
			camera.SetLocation(0, 0);

			GraphicsManager.Update += new GraphicsManager.Updater(GraphicsManager_Update);

			WindowControl glui = new WindowControl(MainCanvas.GetCanvas());
			glui.SetPosition(10, 10);

			Label text = new Label(glui);
			text.Text = "Tests";
			text.SetPosition(10, 10);

			testList = new ListBox(glui);
			testList.RowSelected += delegate(Base sender, ItemSelectedEventArgs tlargs) {
				testSelection = testList.SelectedRowIndex;
			};
			testList.SelectedRowIndex = testSelection;
			testList.SetPosition(10, 30);
			testList.SetSize(170, 60);

			//glui.add_separator();

			NumericUpDown spinner = new NumericUpDown(glui);
			spinner.Text = "Vel Iters";
			spinner.Min = 1;
			spinner.Max = 500;
			spinner.ValueChanged += delegate(Base sender, EventArgs vcargs) {
				settings.velocityIterations = (int)spinner.Value;
			};
			spinner.Value = settings.velocityIterations;
			spinner.SetPosition(10, 100);

			NumericUpDown posSpinner = new NumericUpDown(glui);
			posSpinner.Min = 0;
			posSpinner.Max = 100;
			posSpinner.Text = "Pos Iters";
			posSpinner.ValueChanged += delegate(Base sender, EventArgs psargs) {
				settings.positionIterations = (int)posSpinner.Value;
			};
			posSpinner.Value = settings.positionIterations;
			posSpinner.SetPosition(10, 120);

			NumericUpDown hertzSpinner = new NumericUpDown(glui);
			hertzSpinner.Text = "Hertz";
			hertzSpinner.Min = 5;
			hertzSpinner.Max = 200;
			hertzSpinner.ValueChanged += delegate(Base sender, EventArgs hargs) {
				settingsHz = hertzSpinner.Value;
			};
			hertzSpinner.Value = settingsHz;
			hertzSpinner.SetPosition(10, 140);

			LabeledCheckBox scb = new LabeledCheckBox(glui);
			scb.Text = "Sleep";
			scb.CheckChanged += delegate(Base sender, EventArgs argsscb) {
				settings.enableSleep = scb.IsChecked;
			};
			scb.IsChecked = settings.enableSleep;
			scb.SetPosition(10, 160);

			LabeledCheckBox wsu = new LabeledCheckBox(glui);
			wsu.Text = "Warm Starting";
			wsu.CheckChanged += delegate(Base sender, EventArgs argsscb) {
				settings.enableWarmStarting = wsu.IsChecked;
			};
			wsu.IsChecked = settings.enableWarmStarting;
			wsu.SetPosition(10, 180);

			LabeledCheckBox toi = new LabeledCheckBox(glui);
			toi.Text = "Time of Impact";
			toi.CheckChanged += delegate(Base sender, EventArgs argsscb) {
				settings.enableContinuous = toi.IsChecked;
			};
			toi.IsChecked = settings.enableContinuous;
			toi.SetPosition(10, 200);

			LabeledCheckBox ssb = new LabeledCheckBox(glui);
			ssb.Text = "Sub-Stepping";
			ssb.CheckChanged += delegate(Base sender, EventArgs argsscb) {
				settings.enableSubStepping = ssb.IsChecked;
			};
			ssb.IsChecked = settings.enableSubStepping;
			ssb.SetPosition(10, 220);

			//glui.add_separator();

			//GLUI_Panel* drawPanel =	glui.add_panel("Draw");
			//glui.add_checkbox_to_panel(drawPanel, "Shapes", &settings.drawShapes);
			//glui.add_checkbox_to_panel(drawPanel, "Joints", &settings.drawJoints);
			//glui.add_checkbox_to_panel(drawPanel, "AABBs", &settings.drawAABBs);
			//glui.add_checkbox_to_panel(drawPanel, "Contact Points", &settings.drawContactPoints);
			//glui.add_checkbox_to_panel(drawPanel, "Contact Normals", &settings.drawContactNormals);
			//glui.add_checkbox_to_panel(drawPanel, "Contact Impulses", &settings.drawContactImpulse);
			//glui.add_checkbox_to_panel(drawPanel, "Friction Impulses", &settings.drawFrictionImpulse);
			//glui.add_checkbox_to_panel(drawPanel, "Center of Masses", &settings.drawCOMs);
			//glui.add_checkbox_to_panel(drawPanel, "Statistics", &settings.drawStats);
			//glui.add_checkbox_to_panel(drawPanel, "Profile", &settings.drawProfile);

			foreach (TestEntry e in g_testEntries)
			{
			    testList.AddRow(e.name, "", e);
			}

			//glui.add_button("Pause", 0, Pause);
			//glui.add_button("Single Step", 0, SingleStep);
			//glui.add_button("Restart", 0, Restart);

			//glui.add_button("Quit", 0,(GLUI_Update_CB)Exit);

			glui.SetSize(200, 300);
			GraphicsManager.Start();
		}

		static void GraphicsManager_Update() {
			Keyboard();

			camera.SetLocation(settings.viewCenter.x, settings.viewCenter.y);
			camera.SetZoom(viewZoom);
		}

		static b2Vec2 ConvertScreenToWorld(int x, int y)
		{
			float u = x / tw;
			float v = (th - y) / th;

			float ratio = tw / th;
			b2Vec2 extents = new b2Vec2(ratio * 25.0f, 25.0f);
			extents *= viewZoom;

			b2Vec2 lower = settings.viewCenter - extents;
			b2Vec2 upper = settings.viewCenter + extents;

			b2Vec2 p;
			p.x = (1.0f - u) * lower.x + u * upper.x;
			p.y = (1.0f - v) * lower.y + v * upper.y;
			return p;
		}

		static void SimulationLoop() {
			b2Vec2 oldCenter = settings.viewCenter;
			settings.hz = settingsHz;
			test.Step(settings);

			test.DrawTitle(entry.name);

			if (testSelection != testIndex) {
				testIndex = testSelection;
				entry = g_testEntries[testIndex];
				test = entry.createFcn();
				viewZoom = 12.0f;
				camera.SetZoom(viewZoom);
				settings.viewCenter.Set(0.0f, 20.0f);
			}
		}

		static void Keyboard()
		{
			//esc to quit
			if (KeyboardManager.IsPressed(Key.Escape)){
				Environment.Exit(0);
			}

			//z to zoom out
			if (KeyboardManager.IsPressed(Key.Z)){
				viewZoom = Math.Min(1.1f * viewZoom, 20.0f);
				//camera.SetZoom(viewZoom);
			}

			//x to zoom in
			if (KeyboardManager.IsPressed(Key.X)){
				viewZoom = Math.Max(0.9f * viewZoom, 1f);
				//camera.SetZoom(viewZoom);
			}

			// Press 'r' to reset.
			if (KeyboardManager.IsPressed(Key.R)){
				test = entry.createFcn();
			}

				// Press space to launch a bomb.
			if (KeyboardManager.IsPressed(Key.Space)){
					if (test != null) {
						test.LaunchBomb();
					}
			}
			if (KeyboardManager.IsPressed(Key.P)){
					settings.pause = !settings.pause;
			}

				// Press [ to prev test.
			if (KeyboardManager.IsPressed(Key.BracketLeft)){
					--testSelection;
					if (testSelection < 0) {
						testSelection = testCount - 1;
					}
					testList.SelectedRowIndex = testSelection;
			}

				// Press ] to next test.
			if (KeyboardManager.IsPressed(Key.BracketRight)){
					++testSelection;
					if (testSelection == testCount) {
						testSelection = 0;
					}
					testList.SelectedRowIndex = testSelection;
			}

			    // Press left to pan left.
			if (KeyboardManager.IsDown(Key.Left)){
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        b2Vec2 newOrigin = new b2Vec2(2.0f, 0.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.x += 0.5f;
			    }
			}

			    // Press right to pan right.
			if (KeyboardManager.IsDown(Key.Right)) {
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        b2Vec2 newOrigin = new b2Vec2(-2.0f, 0.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.x -= 0.5f;
			    }
			}

			    // Press down to pan down.
			if (KeyboardManager.IsDown(Key.Down)) {
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        b2Vec2 newOrigin = new b2Vec2(0.0f, 2.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.y += 0.5f;
			    }
			}

			    // Press up to pan up.
			if (KeyboardManager.IsDown(Key.Up)) {
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        b2Vec2 newOrigin = new b2Vec2(0.0f, -2.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.y -= 0.5f;
			    }
			}

			    // Press home to reset the view.
			if (KeyboardManager.IsPressed(Key.Home)){
			    viewZoom = 12.0f;
			    settings.viewCenter.Set(0.0f, 20.0f);
			    //camera.SetZoom(viewZoom);
			}

			test.Keyboard();
		}

		static void Mouse(int button, int state, int x, int y)
		{
			throw new NotImplementedException();
			//// Use the mouse to move things around.
			//if (button == GLUT_LEFT_BUTTON)
			//{
			//    int mod = glutGetModifiers();
			//    b2Vec2 p = ConvertScreenToWorld(x, y);
			//    if (state == GLUT_DOWN)
			//    {
			//        b2Vec2 p = ConvertScreenToWorld(x, y);
			//        if (mod == GLUT_ACTIVE_SHIFT)
			//        {
			//            test.ShiftMouseDown(p);
			//        }
			//        else
			//        {
			//            test.MouseDown(p);
			//        }
			//    }
		
			//    if (state == GLUT_UP)
			//    {
			//        test.MouseUp(p);
			//    }
			//}
			//else if (button == GLUT_RIGHT_BUTTON)
			//{
			//    if (state == GLUT_DOWN)
			//    {	
			//        lastp = ConvertScreenToWorld(x, y);
			//        rMouseDown = true;
			//    }

			//    if (state == GLUT_UP)
			//    {
			//        rMouseDown = false;
			//    }
			//}
		}

		static void MouseMotion(int x, int y)
		{
			throw new NotImplementedException();
			//b2Vec2 p = ConvertScreenToWorld(x, y);
			//test.MouseMove(p);
	
			//if (rMouseDown)
			//{
			//    b2Vec2 diff = p - lastp;
			//    settings.viewCenter.x -= diff.x;
			//    settings.viewCenter.y -= diff.y;
			//    Resize(width, height);
			//    lastp = ConvertScreenToWorld(x, y);
			//}
		}

		static void MouseWheel(int wheel, int direction, int x, int y)
		{
			throw new NotImplementedException();
			//if (direction > 0)
			//{
			//    viewZoom /= 1.1f;
			//}
			//else
			//{
			//    viewZoom *= 1.1f;
			//}
			//Resize(width, height);
		}

		static void Restart()
		{
			throw new NotImplementedException();
			//delete test;
			//entry = g_testEntries + testIndex;
			//test = entry.createFcn();
			//Resize(width, height);
		}

		static void Pause()
		{
			settings.pause = !settings.pause;
		}

		static void Exit(int code)
		{
			Environment.Exit(code);
		}

		static void SingleStep()
		{
			settings.pause = true;
			settings.singleStep = true;
		}
	}
}
