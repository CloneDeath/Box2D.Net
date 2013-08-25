using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	///
	class RopeTest : Test
	{
		public RopeTest()
		{
			const int N = 40;
			Vec2[] vertices = new Vec2[N];
			float [] masses = new float [N];

			for (int i = 0; i < N; ++i)
			{
				vertices[i].Set(0.0f, 20.0f - 0.25f * i);
				masses[i] = 1.0f;
			}
			masses[0] = 0.0f;
			masses[1] = 0.0f;

			RopeDef def = new RopeDef();
			def.vertices = new List<Vec2>(vertices);
			def.count = N;
			def.gravity.Set(0.0f, -10.0f);
			def.masses = new List<float>(masses);
			def.damping = 0.1f;
			def.k2 = 1.0f;
			def.k3 = 0.5f;

			m_rope.Initialize(def);

			m_angle = 0.0f;
			m_rope.SetAngle(m_angle);
		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.Q)){
				m_angle = (float)Math.Max(-(float)Math.PI, m_angle - 0.05f * (float)Math.PI);
				m_rope.SetAngle(m_angle);
			}
			if (KeyboardManager.IsPressed(Key.E)) {
				m_angle = (float)Math.Min(Math.PI, m_angle + 0.05f * (float)Math.PI);
				m_rope.SetAngle(m_angle);
			}
		}

		public override void Step(TestSettings settings)
		{
			float dt = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

			if (settings.pause == true && settings.singleStep == false)
			{
				dt = 0.0f;
			}

			m_rope.Step(dt, 1);

			base.Step(settings);

			m_rope.Draw(m_debugDraw);

			m_debugDraw.DrawString("Press (q,e) to adjust target angle");
			
			m_debugDraw.DrawString("Target angle = %g degrees", m_angle * 180.0f / (float)Math.PI);
			
		}

		public static Test Create()
		{
			return new RopeTest();
		}

		Rope m_rope;
		float m_angle;
	};
}
