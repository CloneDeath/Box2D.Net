﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A weld joint essentially glues two bodies together. A weld joint may
	/// distort somewhat because the island constraint solver is approximate.
	class b2WeldJoint : b2Joint
	{
	
		public override b2Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override b2Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override b2Vec2 GetReactionForce(float inv_dt){
			b2Vec2 P = new b2Vec2(m_impulse.x, m_impulse.y);
			return inv_dt * P;
		}
		public override float GetReactionTorque(float inv_dt){
			return inv_dt * m_impulse.z;
		}

		/// The local anchor point relative to bodyA's origin.
		public b2Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public b2Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// Get the reference angle.
		public float GetReferenceAngle() { return m_referenceAngle; }

		/// Set/get frequency in Hz.
		public void SetFrequency(float hz) { m_frequencyHz = hz; }
		public float GetFrequency() { return m_frequencyHz; }

		/// Set/get damping ratio.
		public void SetDampingRatio(float ratio) { m_dampingRatio = ratio; }
		public float GetDampingRatio() { return m_dampingRatio; }

		/// Dump to b2Settings.b2Log
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			b2Settings.b2Log("  b2WeldJointDef jd;\n");
			b2Settings.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
			b2Settings.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
			b2Settings.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
			b2Settings.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
			b2Settings.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
			b2Settings.b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
			b2Settings.b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
			b2Settings.b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
			b2Settings.b2Log("  joints[%d] = m_world.CreateJoint(&jd);\n", m_index);
		}



		internal b2WeldJoint(b2WeldJointDef def): base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_referenceAngle = def.referenceAngle;
			m_frequencyHz = def.frequencyHz;
			m_dampingRatio = def.dampingRatio;

			m_impulse.SetZero();
		}

		internal override void InitVelocityConstraints(b2SolverData data){
			m_indexA = m_bodyA.m_islandIndex;
			m_indexB = m_bodyB.m_islandIndex;
			m_localCenterA = m_bodyA.m_sweep.localCenter;
			m_localCenterB = m_bodyB.m_sweep.localCenter;
			m_invMassA = m_bodyA.m_invMass;
			m_invMassB = m_bodyB.m_invMass;
			m_invIA = m_bodyA.m_invI;
			m_invIB = m_bodyB.m_invI;

			float aA = data.positions[m_indexA].a;
			b2Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			float aB = data.positions[m_indexB].a;
			b2Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			b2Rot qA = new b2Rot(aA); b2Rot qB = new b2Rot(aB);

			m_rA = Utilities.b2Mul(qA, m_localAnchorA - m_localCenterA);
			m_rB = Utilities.b2Mul(qB, m_localAnchorB - m_localCenterB);

			// J = [-I -r1_skew I r2_skew]
			//     [ 0       -1 0       1]
			// r_skew = [-ry; rx]

			// Matlab
			// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
			//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
			//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			b2Mat33 K;
			K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
			K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
			K.ez.x = -m_rA.y * iA - m_rB.y * iB;
			K.ex.y = K.ey.x;
			K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
			K.ez.y = m_rA.x * iA + m_rB.x * iB;
			K.ex.z = K.ez.x;
			K.ey.z = K.ez.y;
			K.ez.z = iA + iB;

			if (m_frequencyHz > 0.0f)
			{
				K.GetInverse22(out m_mass);

				float invM = iA + iB;
				float m = invM > 0.0f ? 1.0f / invM : 0.0f;

				float C = aB - aA - m_referenceAngle;

				// Frequency
				float omega = 2.0f * (float)Math.PI * m_frequencyHz;

				// Damping coefficient
				float d = 2.0f * m * m_dampingRatio * omega;

				// Spring stiffness
				float k = m * omega * omega;

				// magic formulas
				float h = data.step.dt;
				m_gamma = h * (d + h * k);
				m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
				m_bias = C * h * k * m_gamma;

				invM += m_gamma;
				m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
			}
			else
			{
				K.GetSymInverse33(out m_mass);
				m_gamma = 0.0f;
				m_bias = 0.0f;
			}

			if (data.step.warmStarting)
			{
				// Scale impulses to support a variable time step.
				m_impulse *= data.step.dtRatio;

				b2Vec2 P = new b2Vec2(m_impulse.x, m_impulse.y);

				vA -= mA * P;
				wA -= iA * (Utilities.b2Cross(m_rA, P) + m_impulse.z);

				vB += mB * P;
				wB += iB * (Utilities.b2Cross(m_rB, P) + m_impulse.z);
			}
			else
			{
				m_impulse.SetZero();
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}
		internal override void SolveVelocityConstraints(b2SolverData data){
			b2Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			b2Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			if (m_frequencyHz > 0.0f)
			{
				float Cdot2 = wB - wA;

				float impulse2 = -m_mass.ez.z * (Cdot2 + m_bias + m_gamma * m_impulse.z);
				m_impulse.z += impulse2;

				wA -= iA * impulse2;
				wB += iB * impulse2;

				b2Vec2 Cdot1 = vB + Utilities.b2Cross(wB, m_rB) - vA - Utilities.b2Cross(wA, m_rA);

				b2Vec2 impulse1 = -Utilities.b2Mul22(m_mass, Cdot1);
				m_impulse.x += impulse1.x;
				m_impulse.y += impulse1.y;

				b2Vec2 P = impulse1;

				vA -= mA * P;
				wA -= iA * Utilities.b2Cross(m_rA, P);

				vB += mB * P;
				wB += iB * Utilities.b2Cross(m_rB, P);
			}
			else
			{
				b2Vec2 Cdot1 = vB + Utilities.b2Cross(wB, m_rB) - vA - Utilities.b2Cross(wA, m_rA);
				float Cdot2 = wB - wA;
				b2Vec3 Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

				b2Vec3 impulse = -Utilities.b2Mul(m_mass, Cdot);
				m_impulse += impulse;

				b2Vec2 P = new b2Vec2(impulse.x, impulse.y);

				vA -= mA * P;
				wA -= iA * (Utilities.b2Cross(m_rA, P) + impulse.z);

				vB += mB * P;
				wB += iB * (Utilities.b2Cross(m_rB, P) + impulse.z);
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}
		internal override bool SolvePositionConstraints(b2SolverData data){
			b2Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			b2Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			b2Rot qA = new b2Rot(aA);
			b2Rot qB = new b2Rot(aB);

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			b2Vec2 rA = Utilities.b2Mul(qA, m_localAnchorA - m_localCenterA);
			b2Vec2 rB = Utilities.b2Mul(qB, m_localAnchorB - m_localCenterB);

			float positionError, angularError;

			b2Mat33 K;
			K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
			K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
			K.ez.x = -rA.y * iA - rB.y * iB;
			K.ex.y = K.ey.x;
			K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
			K.ez.y = rA.x * iA + rB.x * iB;
			K.ex.z = K.ez.x;
			K.ey.z = K.ez.y;
			K.ez.z = iA + iB;

			if (m_frequencyHz > 0.0f)
			{
				b2Vec2 C1 =  cB + rB - cA - rA;

				positionError = C1.Length();
				angularError = 0.0f;

				b2Vec2 P = -K.Solve22(C1);

				cA -= mA * P;
				aA -= iA * Utilities.b2Cross(rA, P);

				cB += mB * P;
				aB += iB * Utilities.b2Cross(rB, P);
			}
			else
			{
				b2Vec2 C1 =  cB + rB - cA - rA;
				float C2 = aB - aA - m_referenceAngle;

				positionError = C1.Length();
				angularError = Math.Abs(C2);

				b2Vec3 C = new b2Vec3(C1.x, C1.y, C2);
	
				b2Vec3 impulse = -K.Solve33(C);
				b2Vec2 P = new b2Vec2(impulse.x, impulse.y);

				cA -= mA * P;
				aA -= iA * (Utilities.b2Cross(rA, P) + impulse.z);

				cB += mB * P;
				aB += iB * (Utilities.b2Cross(rB, P) + impulse.z);
			}

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return positionError <=b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
		}

		float m_frequencyHz;
		float m_dampingRatio;
		float m_bias;

		// Solver shared
		b2Vec2 m_localAnchorA;
		b2Vec2 m_localAnchorB;
		float m_referenceAngle;
		float m_gamma;
		b2Vec3 m_impulse;

		// Solver temp
		int m_indexA;
		int m_indexB;
		b2Vec2 m_rA;
		b2Vec2 m_rB;
		b2Vec2 m_localCenterA;
		b2Vec2 m_localCenterB;
		float m_invMassA;
		float m_invMassB;
		float m_invIA;
		float m_invIB;
		b2Mat33 m_mass;
	};
}
