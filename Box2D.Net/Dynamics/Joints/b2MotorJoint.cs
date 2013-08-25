using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A motor joint is used to control the relative motion
	/// between two bodies. A typical usage is to control the movement
	/// of a dynamic body with respect to the ground.
	public class b2MotorJoint : b2Joint
	{
	
		public override b2Vec2 GetAnchorA(){
			return m_bodyA.GetPosition();
		}
		public override b2Vec2 GetAnchorB(){
			return m_bodyB.GetPosition();
		}

		public override b2Vec2 GetReactionForce(float inv_dt){
			return inv_dt * m_linearImpulse;
		}
		public override float GetReactionTorque(float inv_dt){
			return inv_dt * m_angularImpulse;
		}

		/// Set/get the target linear offset, in frame A, in meters.
		public void SetLinearOffset(b2Vec2 linearOffset){
			if (linearOffset.x != m_linearOffset.x || linearOffset.y != m_linearOffset.y)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_linearOffset = linearOffset;
			}
		}
		public b2Vec2 GetLinearOffset() {
			return m_linearOffset;
		}

		/// Set/get the target angular offset, in radians.
		public void SetAngularOffset(float angularOffset){
			if (angularOffset != m_angularOffset)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_angularOffset = angularOffset;
			}
		}
		public float GetAngularOffset(){
			return m_angularOffset;
		}

		/// Set the maximum friction force in N.
		public void SetMaxForce(float force){
			Utilities.Assert(Utilities.IsValid(force) && force >= 0.0f);
			m_maxForce = force;
		}

		/// Get the maximum friction force in N.
		public float GetMaxForce(){
			return m_maxForce;
		}

		/// Set the maximum friction torque in N*m.
		public void SetMaxTorque(float torque){
			Utilities.Assert(Utilities.IsValid(torque) && torque >= 0.0f);
			m_maxTorque = torque;
		}

		/// Get the maximum friction torque in N*m.
		public float GetMaxTorque(){
			return m_maxTorque;
		}

		/// Dump to b2Settings.b2Log
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			b2Settings.b2Log("  b2MotorJointDef jd;\n");
			b2Settings.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
			b2Settings.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
			b2Settings.b2Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			b2Settings.b2Log("  jd.linearOffset.Set(%.15lef, %.15lef);\n", m_linearOffset.x, m_linearOffset.y);
			b2Settings.b2Log("  jd.angularOffset = %.15lef;\n", m_angularOffset);
			b2Settings.b2Log("  jd.maxForce = %.15lef;\n", m_maxForce);
			b2Settings.b2Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
			b2Settings.b2Log("  jd.correctionFactor = %.15lef;\n", m_correctionFactor);
			b2Settings.b2Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}


		internal b2MotorJoint(b2MotorJointDef def): base(def)
		{
			m_linearOffset = def.linearOffset;
			m_angularOffset = def.angularOffset;

			m_linearImpulse.SetZero();
			m_angularImpulse = 0.0f;

			m_maxForce = def.maxForce;
			m_maxTorque = def.maxTorque;
			m_correctionFactor = def.correctionFactor;
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

			b2Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			b2Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			b2Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			b2Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			b2Rot qA = new b2Rot(aA); b2Rot qB = new b2Rot(aB);

			// Compute the effective mass matrix.
			m_rA = Utilities.b2Mul(qA, -m_localCenterA);
			m_rB = Utilities.b2Mul(qB, -m_localCenterB);

			// J = [-I -r1_skew I r2_skew]
			//     [ 0       -1 0       1]
			// r_skew = [-ry; rx]

			// Matlab
			// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
			//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
			//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			b2Mat22 K;
			K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
			K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
			K.ey.x = K.ex.y;
			K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

			m_linearMass = K.GetInverse();

			m_angularMass = iA + iB;
			if (m_angularMass > 0.0f)
			{
				m_angularMass = 1.0f / m_angularMass;
			}

			m_linearError = cB + m_rB - cA - m_rA - Utilities.b2Mul(qA, m_linearOffset);
			m_angularError = aB - aA - m_angularOffset;

			if (data.step.warmStarting)
			{
				// Scale impulses to support a variable time step.
				m_linearImpulse *= data.step.dtRatio;
				m_angularImpulse *= data.step.dtRatio;

				b2Vec2 P = new b2Vec2(m_linearImpulse.x, m_linearImpulse.y);
				vA -= mA * P;
				wA -= iA * (Utilities.b2Cross(m_rA, P) + m_angularImpulse);
				vB += mB * P;
				wB += iB * (Utilities.b2Cross(m_rB, P) + m_angularImpulse);
			}
			else
			{
				m_linearImpulse.SetZero();
				m_angularImpulse = 0.0f;
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

			float h = data.step.dt;
			float inv_h = data.step.inv_dt;

			// Solve angular friction
			{
				float Cdot = wB - wA + inv_h * m_correctionFactor * m_angularError;
				float impulse = -m_angularMass * Cdot;

				float oldImpulse = m_angularImpulse;
				float maxImpulse = h * m_maxTorque;
				m_angularImpulse = Utilities.b2Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = m_angularImpulse - oldImpulse;

				wA -= iA * impulse;
				wB += iB * impulse;
			}

			// Solve linear friction
			{
				b2Vec2 Cdot = vB + Utilities.b2Cross(wB, m_rB) - vA - Utilities.b2Cross(wA, m_rA) + inv_h * m_correctionFactor * m_linearError;

				b2Vec2 impulse = -Utilities.b2Mul(m_linearMass, Cdot);
				b2Vec2 oldImpulse = m_linearImpulse;
				m_linearImpulse += impulse;

				float maxImpulse = h * m_maxForce;

				if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
				{
					m_linearImpulse.Normalize();
					m_linearImpulse *= maxImpulse;
				}

				impulse = m_linearImpulse - oldImpulse;

				vA -= mA * impulse;
				wA -= iA * Utilities.b2Cross(m_rA, impulse);

				vB += mB * impulse;
				wB += iB * Utilities.b2Cross(m_rB, impulse);
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}
		internal override bool SolvePositionConstraints(b2SolverData data){

			return true;
		}

		// Solver shared
		b2Vec2 m_linearOffset;
		float m_angularOffset;
		b2Vec2 m_linearImpulse;
		float m_angularImpulse;
		float m_maxForce;
		float m_maxTorque;
		float m_correctionFactor;

		// Solver temp
		int m_indexA;
		int m_indexB;
		b2Vec2 m_rA;
		b2Vec2 m_rB;
		b2Vec2 m_localCenterA;
		b2Vec2 m_localCenterB;
		b2Vec2 m_linearError;
		float m_angularError;
		float m_invMassA;
		float m_invMassB;
		float m_invIA;
		float m_invIB;
		b2Mat22 m_linearMass;
		float m_angularMass;
	};
}
