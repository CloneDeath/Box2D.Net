using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Testbed.Framework;
using Box2D;

namespace Testbed.Tests {
	///
	class Rope : Test
	{
		public Rope()
		{
			const int N = 40;
			b2Vec2 vertices[N];
			float masses[N];

			for (int i = 0; i < N; ++i)
			{
				vertices[i].Set(0.0f, 20.0f - 0.25f * i);
				masses[i] = 1.0f;
			}
			masses[0] = 0.0f;
			masses[1] = 0.0f;

			b2RopeDef def;
			def.vertices = vertices;
			def.count = N;
			def.gravity.Set(0.0f, -10.0f);
			def.masses = masses;
			def.damping = 0.1f;
			def.k2 = 1.0f;
			def.k3 = 0.5f;

			m_rope.Initialize(&def);

			m_angle = 0.0f;
			m_rope.SetAngle(m_angle);
		}

		public void Keyboard()
		{
			switch (key)
			{
			case 'q':
				m_angle = Math.Max(-Math.PI, m_angle - 0.05f * (float)Math.PI);
				m_rope.SetAngle(m_angle);
				break;

			case 'e':
				m_angle = Math.Min(Math.PI, m_angle + 0.05f * (float)Math.PI);
				m_rope.SetAngle(m_angle);
				break;
			}
		}

		public override void Step(Settings settings)
		{
			float dt = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

			if (settings.pause == 1 && settings.singleStep == 0)
			{
				dt = 0.0f;
			}

			m_rope.Step(dt, 1);

			base.Step(settings);

			m_rope.Draw(&m_debugDraw);

			m_debugDraw.DrawString("Press (q,e) to adjust target angle");
			
			m_debugDraw.DrawString("Target angle = %g degrees", m_angle * 180.0f / (float)Math.PI);
			
		}

		public static Test Create()
		{
			return new Rope();
		}

		b2Rope m_rope;
		float m_angle;
	};
}
