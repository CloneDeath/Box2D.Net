using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace Box2D {
	public class Rope {
		private int m_count;
		private List<Vec2> m_ps;
		private Vec2 m_p0s; //was pointer
		private Vec2 m_vs; //was pointer

		private float m_ims; //was pointer

		private float m_Ls; //was pointer
		private float m_as; //was pointer

		private Vec2 m_gravity;
		private float m_damping;

		private float m_k2;
		private float m_k3;

		public Rope(){
			throw new NotImplementedException();
			//m_count = 0;
			//m_ps = new List<Vec2>();
			//m_p0s = null;
			//m_vs = null;
			//m_ims = null;
			//m_Ls = null;
			//m_as = null;
			//m_gravity.SetZero();
			//m_k2 = 1.0f;
			//m_k3 = 0.1f;
		}

		~Rope()
		{
			throw new NotImplementedException();
			//Free(m_ps);
			//Free(m_p0s);
			//Free(m_vs);
			//Free(m_ims);
			//Free(m_Ls);
			//Free(m_as);
		}

		///
		public void Initialize(RopeDef def){
			throw new NotImplementedException();
			//Utilities.Assert(def.count >= 3);
			//m_count = def.count;
			//m_ps = (Vec2*)Alloc(m_count * sizeof(Vec2));
			//m_p0s = (Vec2*)Alloc(m_count * sizeof(Vec2));
			//m_vs = (Vec2*)Alloc(m_count * sizeof(Vec2));
			//m_ims = (float*)Alloc(m_count * sizeof(float));

			//for (int i = 0; i < m_count; ++i)
			//{
			//    m_ps[i] = def.vertices[i];
			//    m_p0s[i] = def.vertices[i];
			//    m_vs[i].SetZero();

			//    float m = def.masses[i];
			//    if (m > 0.0f)
			//    {
			//        m_ims[i] = 1.0f / m;
			//    }
			//    else
			//    {
			//        m_ims[i] = 0.0f;
			//    }
			//}

			//int count2 = m_count - 1;
			//int count3 = m_count - 2;
			//m_Ls = (float*)Alloc(count2 * sizeof(float));
			//m_as = (float*)Alloc(count3 * sizeof(float));

			//for (int i = 0; i < count2; ++i)
			//{
			//    Vec2 p1 = m_ps[i];
			//    Vec2 p2 = m_ps[i+1];
			//    m_Ls[i] = Distance(p1, p2);
			//}

			//for (int i = 0; i < count3; ++i)
			//{
			//    Vec2 p1 = m_ps[i];
			//    Vec2 p2 = m_ps[i + 1];
			//    Vec2 p3 = m_ps[i + 2];

			//    Vec2 d1 = p2 - p1;
			//    Vec2 d2 = p3 - p2;

			//    float a = Utilities.Cross(d1, d2);
			//    float b = Utilities.Dot(d1, d2);

			//    m_as[i] = Atan2(a, b);
			//}

			//m_gravity = def.gravity;
			//m_damping = def.damping;
			//m_k2 = def.k2;
			//m_k3 = def.k3;
		}

		///
		public void Step(float timeStep, int iterations){
			throw new NotImplementedException();
			//if (h == 0.0)
			//{
			//    return;
			//}

			//float d = expf(- h * m_damping);

			//for (int i = 0; i < m_count; ++i)
			//{
			//    m_p0s[i] = m_ps[i];
			//    if (m_ims[i] > 0.0f)
			//    {
			//        m_vs[i] += h * m_gravity;
			//    }
			//    m_vs[i] *= d;
			//    m_ps[i] += h * m_vs[i];

			//}

			//for (int i = 0; i < iterations; ++i)
			//{
			//    SolveC2();
			//    SolveC3();
			//    SolveC2();
			//}

			//float inv_h = 1.0f / h;
			//for (int i = 0; i < m_count; ++i)
			//{
			//    m_vs[i] = inv_h * (m_ps[i] - m_p0s[i]);
			//}
		}

		///
		public int GetVertexCount()
		{
			return m_count;
		}

		///
		public List<Vec2> GetVertices()
		{
			return m_ps;
		}

		///
		public void Draw(Draw draw){
			Color c = Color.FromArgb(102, 127, 179);

			for (int i = 0; i < m_count - 1; ++i)
			{
				draw.DrawSegment(m_ps[i], m_ps[i+1], c);
			}
		}

		///
		public void SetAngle(float angle){
			throw new NotImplementedException();
			//int count3 = m_count - 2;
			//for (int i = 0; i < count3; ++i)
			//{
			//    m_as[i] = angle;
			//}
		}



		private void SolveC2(){
			throw new NotImplementedException();
			//int count2 = m_count - 1;

			//for (int i = 0; i < count2; ++i)
			//{
			//    Vec2 p1 = m_ps[i];
			//    Vec2 p2 = m_ps[i + 1];

			//    Vec2 d = p2 - p1;
			//    float L = d.Normalize();

			//    float im1 = m_ims[i];
			//    float im2 = m_ims[i + 1];

			//    if (im1 + im2 == 0.0f)
			//    {
			//        continue;
			//    }

			//    float s1 = im1 / (im1 + im2);
			//    float s2 = im2 / (im1 + im2);

			//    p1 -= m_k2 * s1 * (m_Ls[i] - L) * d;
			//    p2 += m_k2 * s2 * (m_Ls[i] - L) * d;

			//    m_ps[i] = p1;
			//    m_ps[i + 1] = p2;
			//}
		}

		private void SolveC3(){
			throw new NotImplementedException();
			//int count3 = m_count - 2;

			//for (int i = 0; i < count3; ++i) {
			//    Vec2 p1 = m_ps[i];
			//    Vec2 p2 = m_ps[i + 1];
			//    Vec2 p3 = m_ps[i + 2];

			//    float m1 = m_ims[i];
			//    float m2 = m_ims[i + 1];
			//    float m3 = m_ims[i + 2];

			//    Vec2 d1 = p2 - p1;
			//    Vec2 d2 = p3 - p2;

			//    float L1sqr = d1.LengthSquared();
			//    float L2sqr = d2.LengthSquared();

			//    if (L1sqr * L2sqr == 0.0f) {
			//        continue;
			//    }

			//    float a = Utilities.Cross(d1, d2);
			//    float b = Utilities.Dot(d1, d2);

			//    float angle = Atan2(a, b);

			//    Vec2 Jd1 = (-1.0f / L1sqr) * d1.Skew();
			//    Vec2 Jd2 = (1.0f / L2sqr) * d2.Skew();

			//    Vec2 J1 = -Jd1;
			//    Vec2 J2 = Jd1 - Jd2;
			//    Vec2 J3 = Jd2;

			//    float mass = m1 * Utilities.Dot(J1, J1) + m2 * Utilities.Dot(J2, J2) + m3 * Utilities.Dot(J3, J3);
			//    if (mass == 0.0f) {
			//        continue;
			//    }

			//    mass = 1.0f / mass;

			//    float C = angle - m_as[i];

			//    while (C > (float)Math.PI) {
			//        angle -= 2 * (float)Math.PI;
			//        C = angle - m_as[i];
			//    }

			//    while (C < -(float)Math.PI) {
			//        angle += 2.0f * (float)Math.PI;
			//        C = angle - m_as[i];
			//    }

			//    float impulse = -m_k3 * mass * C;

			//    p1 += (m1 * impulse) * J1;
			//    p2 += (m2 * impulse) * J2;
			//    p3 += (m3 * impulse) * J3;

			//    m_ps[i] = p1;
			//    m_ps[i + 1] = p2;
			//    m_ps[i + 2] = p3;
			//}
		}
	}
}
