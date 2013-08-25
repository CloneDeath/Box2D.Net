using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A weld joint essentially glues two bodies together. A weld joint may
	/// distort somewhat because the island constraint solver is approximate.
	public class WeldJoint : Joint
	{
	
		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override Vec2 GetReactionForce(float inv_dt){
			Vec2 P = new Vec2(m_impulse.X, m_impulse.Y);
			return inv_dt * P;
		}
		public override float GetReactionTorque(float inv_dt){
			return inv_dt * m_impulse.Z;
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// Get the reference angle.
		public float GetReferenceAngle() { return m_referenceAngle; }

		/// Set/get frequency in Hz.
		public void SetFrequency(float hz) { m_frequencyHz = hz; }
		public float GetFrequency() { return m_frequencyHz; }

		/// Set/get damping ratio.
		public void SetDampingRatio(float ratio) { m_dampingRatio = ratio; }
		public float GetDampingRatio() { return m_dampingRatio; }

		/// Dump to Settings.Log
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  WeldJointDef jd = new WeldJointDef();\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.X, m_localAnchorA.Y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.X, m_localAnchorB.Y);
			Settings.Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
			Settings.Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
			Settings.Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}



		internal WeldJoint(WeldJointDef def): base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_referenceAngle = def.referenceAngle;
			m_frequencyHz = def.frequencyHz;
			m_dampingRatio = def.dampingRatio;

			m_impulse.SetZero();
		}

		internal override void InitVelocityConstraints(SolverData data){
			m_indexA = m_bodyA.m_islandIndex;
			m_indexB = m_bodyB.m_islandIndex;
			m_localCenterA = m_bodyA.m_sweep.localCenter;
			m_localCenterB = m_bodyB.m_sweep.localCenter;
			m_invMassA = m_bodyA.m_invMass;
			m_invMassB = m_bodyB.m_invMass;
			m_invIA = m_bodyA.m_invI;
			m_invIB = m_bodyB.m_invI;

			float aA = data.positions[m_indexA].a;
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			float aB = data.positions[m_indexB].a;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			Rot qA = new Rot(aA); Rot qB = new Rot(aB);

			m_rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			m_rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);

			// J = [-I -r1_skew I r2_skew]
			//     [ 0       -1 0       1]
			// r_skew = [-ry; rx]

			// Matlab
			// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
			//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
			//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			Mat33 K;
			K.ex.X = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
			K.ey.X = -m_rA.Y * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
			K.ez.X = -m_rA.Y * iA - m_rB.Y * iB;
			K.ex.Y = K.ey.X;
			K.ey.Y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
			K.ez.Y = m_rA.X * iA + m_rB.X * iB;
			K.ex.Z = K.ez.X;
			K.ey.Z = K.ez.Y;
			K.ez.Z = iA + iB;

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
				m_mass.ez.Z = invM != 0.0f ? 1.0f / invM : 0.0f;
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

				Vec2 P = new Vec2(m_impulse.X, m_impulse.Y);

				vA -= mA * P;
				wA -= iA * (Utilities.Cross(m_rA, P) + m_impulse.Z);

				vB += mB * P;
				wB += iB * (Utilities.Cross(m_rB, P) + m_impulse.Z);
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
		internal override void SolveVelocityConstraints(SolverData data){
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			if (m_frequencyHz > 0.0f)
			{
				float Cdot2 = wB - wA;

				float impulse2 = -m_mass.ez.Z * (Cdot2 + m_bias + m_gamma * m_impulse.Z);
				m_impulse.Z += impulse2;

				wA -= iA * impulse2;
				wB += iB * impulse2;

				Vec2 Cdot1 = vB + Utilities.Cross(wB, m_rB) - vA - Utilities.Cross(wA, m_rA);

				Vec2 impulse1 = -Utilities.Mul22(m_mass, Cdot1);
				m_impulse.X += impulse1.X;
				m_impulse.Y += impulse1.Y;

				Vec2 P = impulse1;

				vA -= mA * P;
				wA -= iA * Utilities.Cross(m_rA, P);

				vB += mB * P;
				wB += iB * Utilities.Cross(m_rB, P);
			}
			else
			{
				Vec2 Cdot1 = vB + Utilities.Cross(wB, m_rB) - vA - Utilities.Cross(wA, m_rA);
				float Cdot2 = wB - wA;
				Vec3 Cdot = new Vec3(Cdot1.X, Cdot1.Y, Cdot2);

				Vec3 impulse = -Utilities.Mul(m_mass, Cdot);
				m_impulse += impulse;

				Vec2 P = new Vec2(impulse.X, impulse.Y);

				vA -= mA * P;
				wA -= iA * (Utilities.Cross(m_rA, P) + impulse.Z);

				vB += mB * P;
				wB += iB * (Utilities.Cross(m_rB, P) + impulse.Z);
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}
		internal override bool SolvePositionConstraints(SolverData data){
			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			Rot qA = new Rot(aA);
			Rot qB = new Rot(aB);

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);

			float positionError, angularError;

			Mat33 K;
			K.ex.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
			K.ey.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
			K.ez.X = -rA.Y * iA - rB.Y * iB;
			K.ex.Y = K.ey.X;
			K.ey.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
			K.ez.Y = rA.X * iA + rB.X * iB;
			K.ex.Z = K.ez.X;
			K.ey.Z = K.ez.Y;
			K.ez.Z = iA + iB;

			if (m_frequencyHz > 0.0f)
			{
				Vec2 C1 =  cB + rB - cA - rA;

				positionError = C1.Length();
				angularError = 0.0f;

				Vec2 P = -K.Solve22(C1);

				cA -= mA * P;
				aA -= iA * Utilities.Cross(rA, P);

				cB += mB * P;
				aB += iB * Utilities.Cross(rB, P);
			}
			else
			{
				Vec2 C1 =  cB + rB - cA - rA;
				float C2 = aB - aA - m_referenceAngle;

				positionError = C1.Length();
				angularError = Math.Abs(C2);

				Vec3 C = new Vec3(C1.X, C1.Y, C2);
	
				Vec3 impulse = -K.Solve33(C);
				Vec2 P = new Vec2(impulse.X, impulse.Y);

				cA -= mA * P;
				aA -= iA * (Utilities.Cross(rA, P) + impulse.Z);

				cB += mB * P;
				aB += iB * (Utilities.Cross(rB, P) + impulse.Z);
			}

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return positionError <=Settings._linearSlop && angularError <= Settings._angularSlop;
		}

		float m_frequencyHz;
		float m_dampingRatio;
		float m_bias;

		// Solver shared
		Vec2 m_localAnchorA;
		Vec2 m_localAnchorB;
		float m_referenceAngle;
		float m_gamma;
		Vec3 m_impulse;

		// Solver temp
		int m_indexA;
		int m_indexB;
		Vec2 m_rA;
		Vec2 m_rB;
		Vec2 m_localCenterA;
		Vec2 m_localCenterB;
		float m_invMassA;
		float m_invMassB;
		float m_invIA;
		float m_invIB;
		Mat33 m_mass;
	};
}
