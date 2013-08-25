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
		static TestSettings settings = new TestSettings();
		static int width = 800;
		static int height = 600;
		static int framePeriod = 16;
		static int mainWindow;
		static float settingsHz = 60.0f;
		static float viewZoom = 12.0f;
		static int tx, ty, tw, th;
		static bool rMouseDown;
		static Vec2 lastp;

		static Camera2D camera;
		static ListBox testList;
		public static void Main(string[] args) {
			g_testEntries = AllTests.GetTests();
			testCount = g_testEntries.Count();

			testIndex = Math.Max(0, Math.Min(testIndex, testCount - 1));
			testSelection = testIndex;

			entry = g_testEntries[testIndex];
			test = entry.createFcn();

			GraphicsManager.SetWindowState(OpenTK.WindowState.Maximized);
			string title = String.Format("Box2D Version {0}.{1}.{2}", Settings._version.major, Settings._version.minor, Settings._version.revision);
			GraphicsManager.SetTitle(title);

			camera = new Camera2D();
			camera.OnRender += SimulationLoop;

			camera.SetZoom(12);
			camera.CenterOnTarget(true);
			camera.SetLocation(0, 0);

			GraphicsManager.Update += new GraphicsManager.Updater(GraphicsManager_Update);

			WindowControl glui = new WindowControl(MainCanvas.GetCanvas());
			glui.Dock = Gwen.Pos.Left;

			Label text = new Label(glui);
			text.Text = "Tests";
			text.SetPosition(10, 10);

			testList = new ListBox(glui);
			testList.RowSelected += delegate(Base sender, ItemSelectedEventArgs tlargs) {
				testSelection = testList.SelectedRowIndex;
			};
			foreach (TestEntry e in g_testEntries) {
				testList.AddRow(e.name, "", e);
			}
			testList.SelectedRowIndex = testSelection;
			testList.SetPosition(10, 30);
			testList.SetSize(170, 180);

			//glui.add_separator();
			Base SettingsBox = new Base(glui);
			SettingsBox.SetSize(200, 185);
			SettingsBox.SetPosition(0, 250);
			{
				NumericUpDown spinner = new NumericUpDown(SettingsBox);
				spinner.Text = "Vel Iters";
				spinner.Min = 1;
				spinner.Max = 500;
				spinner.ValueChanged += delegate(Base sender, EventArgs vcargs) {
					settings.velocityIterations = (int)spinner.Value;
				};
				spinner.Value = settings.velocityIterations;
				spinner.SetPosition(10, 10);

				NumericUpDown posSpinner = new NumericUpDown(SettingsBox);
				posSpinner.Min = 0;
				posSpinner.Max = 100;
				posSpinner.Text = "Pos Iters";
				posSpinner.ValueChanged += delegate(Base sender, EventArgs psargs) {
					settings.positionIterations = (int)posSpinner.Value;
				};
				posSpinner.Value = settings.positionIterations;
				posSpinner.SetPosition(10, 35);

				NumericUpDown hertzSpinner = new NumericUpDown(SettingsBox);
				hertzSpinner.Text = "Hertz";
				hertzSpinner.Min = 5;
				hertzSpinner.Max = 200;
				hertzSpinner.ValueChanged += delegate(Base sender, EventArgs hargs) {
					settingsHz = hertzSpinner.Value;
				};
				hertzSpinner.Value = settingsHz;
				hertzSpinner.SetPosition(10, 60);

				LabeledCheckBox scb = new LabeledCheckBox(SettingsBox);
				scb.Text = "Sleep";
				scb.CheckChanged += delegate(Base sender, EventArgs argsscb) {
					settings.enableSleep = scb.IsChecked;
				};
				scb.IsChecked = settings.enableSleep;
				scb.SetPosition(10, 85);

				LabeledCheckBox wsu = new LabeledCheckBox(SettingsBox);
				wsu.Text = "Warm Starting";
				wsu.CheckChanged += delegate(Base sender, EventArgs argsscb) {
					settings.enableWarmStarting = wsu.IsChecked;
				};
				wsu.IsChecked = settings.enableWarmStarting;
				wsu.SetPosition(10, 110);

				LabeledCheckBox toi = new LabeledCheckBox(SettingsBox);
				toi.Text = "Time of Impact";
				toi.CheckChanged += delegate(Base sender, EventArgs argsscb) {
					settings.enableContinuous = toi.IsChecked;
				};
				toi.IsChecked = settings.enableContinuous;
				toi.SetPosition(10, 135);

				LabeledCheckBox ssb = new LabeledCheckBox(SettingsBox);
				ssb.Text = "Sub-Stepping";
				ssb.CheckChanged += delegate(Base sender, EventArgs argsscb) {
					settings.enableSubStepping = ssb.IsChecked;
				};
				ssb.IsChecked = settings.enableSubStepping;
				ssb.SetPosition(10, 160);
			}

			Base drawPanel = new Base(glui);
			drawPanel.Dock = Gwen.Pos.Bottom;
			drawPanel.SetSize(200, 225);
			{
				LabeledCheckBox cbShapes = new LabeledCheckBox(drawPanel);
				cbShapes.Text = "Shapes";
				cbShapes.IsChecked = settings.drawShapes;
				cbShapes.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawShapes = cbShapes.IsChecked;
				};
				cbShapes.SetPosition(10, 10);



				//glui.add_checkbox_to_panel(drawPanel, "Joints", &settings.drawJoints);
				LabeledCheckBox cbJoints = new LabeledCheckBox(drawPanel);
				cbJoints.Text = "Joints";
				cbJoints.IsChecked = settings.drawJoints;
				cbJoints.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawJoints = cbJoints.IsChecked;
				};
				cbJoints.SetPosition(10, 30);



				//glui.add_checkbox_to_panel(drawPanel, "AABBs", &settings.drawAABBs);
				LabeledCheckBox cbAABBs = new LabeledCheckBox(drawPanel);
				cbAABBs.Text = "AABBs";
				cbAABBs.IsChecked = settings.drawAABBs;
				cbAABBs.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawAABBs = cbAABBs.IsChecked;
				};
				cbAABBs.SetPosition(10, 50);



				//glui.add_checkbox_to_panel(drawPanel, "Contact Points", &settings.drawContactPoints);
				LabeledCheckBox cbPoints = new LabeledCheckBox(drawPanel);
				cbPoints.Text = "Contact Points";
				cbPoints.IsChecked = settings.drawContactPoints;
				cbPoints.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawContactPoints = cbPoints.IsChecked;
				};
				cbPoints.SetPosition(10, 70);



				//glui.add_checkbox_to_panel(drawPanel, "Contact Normals", &settings.drawContactNormals);
				LabeledCheckBox cbNormals = new LabeledCheckBox(drawPanel);
				cbNormals.Text = "Contact Normals";
				cbNormals.IsChecked = settings.drawContactNormals;
				cbNormals.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawContactNormals = cbNormals.IsChecked;
				};
				cbNormals.SetPosition(10, 90);



				//glui.add_checkbox_to_panel(drawPanel, "Contact Impulses", &settings.drawContactImpulse);
				LabeledCheckBox cbImpulses = new LabeledCheckBox(drawPanel);
				cbImpulses.Text = "Contact Impulses";
				cbImpulses.IsChecked = settings.drawContactImpulse;
				cbImpulses.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawContactImpulse = cbImpulses.IsChecked;
				};
				cbImpulses.SetPosition(10, 110);



				//glui.add_checkbox_to_panel(drawPanel, "Friction Impulses", &settings.drawFrictionImpulse);
				LabeledCheckBox cbFriction = new LabeledCheckBox(drawPanel);
				cbFriction.Text = "Friction Impulses";
				cbFriction.IsChecked = settings.drawFrictionImpulse;
				cbFriction.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawFrictionImpulse = cbFriction.IsChecked;
				};
				cbFriction.SetPosition(10, 130);



				//glui.add_checkbox_to_panel(drawPanel, "Center of Masses", &settings.drawCOMs);
				LabeledCheckBox cbMasses = new LabeledCheckBox(drawPanel);
				cbMasses.Text = "Center of Masses";
				cbMasses.IsChecked = settings.drawCOMs;
				cbMasses.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawCOMs = cbMasses.IsChecked;
				};
				cbMasses.SetPosition(10, 150);



				//glui.add_checkbox_to_panel(drawPanel, "Statistics", &settings.drawStats);
				LabeledCheckBox cbStatistics = new LabeledCheckBox(drawPanel);
				cbStatistics.Text = "Statistics";
				cbStatistics.IsChecked = settings.drawStats;
				cbStatistics.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawStats = cbStatistics.IsChecked;
				};
				cbStatistics.SetPosition(10, 170);



				//glui.add_checkbox_to_panel(drawPanel, "Profile", &settings.drawProfile);
				LabeledCheckBox cbProfile = new LabeledCheckBox(drawPanel);
				cbProfile.Text = "Profile";
				cbProfile.IsChecked = settings.drawProfile;
				cbProfile.CheckChanged += delegate(Base cbshapes, EventArgs eacbshapes) {
					settings.drawProfile = cbProfile.IsChecked;
				};
				cbProfile.SetPosition(10, 190);
			}


			Base Buttons = new Base(glui);
			Buttons.Dock = Gwen.Pos.Bottom;
			Buttons.Height = 100;
			{
				Button btnPause = new Button(Buttons);
				btnPause.Text = "Pause";
				btnPause.IsToggle = true;
				btnPause.SetPosition(10, 10);
				btnPause.ToggleState = settings.pause;
				btnPause.Clicked += delegate(Base sender, ClickedEventArgs evargs) {
					settings.pause = btnPause.ToggleState;
				};

				Button btnSS = new Button(Buttons);
				btnSS.Text = "Single Step";
				btnSS.SetPosition(10, 40);
				btnSS.Clicked += delegate(Base sender, ClickedEventArgs evargs) {
					SingleStep();
				};

				Button btnRestart = new Button(Buttons);
				btnRestart.Text = "Restart";
				btnRestart.SetPosition(10, 70);
				btnRestart.Clicked += delegate(Base sender, ClickedEventArgs evargs) {
					Restart();
				};
			}

			glui.SetSize(200, 300);			
			GraphicsManager.Start();
		}

		static void GraphicsManager_Update() {
			Keyboard();
			MouseWheel(MouseManager.GetMouseWheel());
			//MouseMotion((int)MouseManager.GetMousePosition().X, (int)MouseManager.GetMousePosition().Y);

			camera.SetLocation(settings.viewCenter.X, settings.viewCenter.Y);
			camera.SetZoom(viewZoom);
		}

		static Vec2 ConvertScreenToWorld(int x, int y)
		{
			float u = x / tw;
			float v = (th - y) / th;

			float ratio = tw / th;
			Vec2 extents = new Vec2(ratio * 25.0f, 25.0f);
			extents *= viewZoom;

			Vec2 lower = settings.viewCenter - extents;
			Vec2 upper = settings.viewCenter + extents;

			Vec2 p;
			p.X = (1.0f - u) * lower.X + u * upper.X;
			p.Y = (1.0f - v) * lower.Y + v * upper.Y;
			return p;
		}

		static void SimulationLoop() {
			Vec2 oldCenter = settings.viewCenter;
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
			        Vec2 newOrigin = new Vec2(2.0f, 0.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.X += 0.5f;
			    }
			}

			    // Press right to pan right.
			if (KeyboardManager.IsDown(Key.Right)) {
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        Vec2 newOrigin = new Vec2(-2.0f, 0.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.X -= 0.5f;
			    }
			}

			    // Press down to pan down.
			if (KeyboardManager.IsDown(Key.Down)) {
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        Vec2 newOrigin = new Vec2(0.0f, 2.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.Y += 0.5f;
			    }
			}

			    // Press up to pan up.
			if (KeyboardManager.IsDown(Key.Up)) {
			    if ( KeyboardManager.IsDown(Key.LControl) || KeyboardManager.IsDown(Key.RControl))
			    {
			        Vec2 newOrigin = new Vec2(0.0f, -2.0f);
			        test.ShiftOrigin(newOrigin);
			    }
			    else
			    {
			        settings.viewCenter.Y -= 0.5f;
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
			//    Vec2 p = ConvertScreenToWorld(x, y);
			//    if (state == GLUT_DOWN)
			//    {
			//        Vec2 p = ConvertScreenToWorld(x, y);
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
			Vec2 p = ConvertScreenToWorld(x, y);
			test.MouseMove(p);

			if (rMouseDown) {
				Vec2 diff = p - lastp;
				settings.viewCenter.X -= diff.X;
				settings.viewCenter.Y -= diff.Y;
				lastp = ConvertScreenToWorld(x, y);
			}
		}

		static void MouseWheel(int direction)
		{
			if (direction > 0) {
				viewZoom /= 1.1f;
			} else if (direction < 0) {
			    viewZoom *= 1.1f;
			}
		}

		static void Restart()
		{
			entry = g_testEntries[testIndex];
			test = entry.createFcn();
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
