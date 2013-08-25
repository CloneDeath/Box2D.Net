using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Friction joint. This is used for top-down friction.
	/// It provides 2D translational friction and angular friction.
	public class FrictionJoint : Joint
	{
		public override Vec2 GetAnchorA() {
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB() {
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override Vec2 GetReactionForce(float inv_dt) {
			return inv_dt * m_linearImpulse;
		}
		public override float GetReactionTorque(float inv_dt) {
			return inv_dt * m_angularImpulse;
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public Vec2 GetLocalAnchorB() { return m_localAnchorB; }

		/// Set the maximum friction force in N.
		public void SetMaxForce(float force) {
			Utilities.Assert(Utilities.IsValid(force) && force >= 0.0f);
			m_maxForce = force;
		}

		/// Get the maximum friction force in N.
		public float GetMaxForce() {
			return m_maxForce;
		}

		/// Set the maximum friction torque in N*m.
		public void SetMaxTorque(float torque) {
			Utilities.Assert(Utilities.IsValid(torque) && torque >= 0.0f);
			m_maxTorque = torque;
		}

		/// Get the maximum friction torque in N*m.
		public float GetMaxTorque() {
			return m_maxTorque;
		}

		/// Dump joint to dmLog
		public override void Dump() {
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  FrictionJointDef jd;\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.X, m_localAnchorA.Y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.X, m_localAnchorB.Y);
			Settings.Log("  jd.maxForce = %.15lef;\n", m_maxForce);
			Settings.Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}

		internal FrictionJoint(FrictionJointDef def) : base(def) {
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;

			m_linearImpulse.SetZero();
			m_angularImpulse = 0.0f;

			m_maxForce = def.maxForce;
			m_maxTorque = def.maxTorque;
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

			Rot qA = new Rot(aA);
			Rot qB = new Rot(aB);

			// Compute the effective mass matrix.
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

			Mat22 K;
			K.ex.X = mA + mB + iA * m_rA.Y * m_rA.Y + iB * m_rB.Y * m_rB.Y;
			K.ex.Y = -iA * m_rA.X * m_rA.Y - iB * m_rB.X * m_rB.Y;
			K.ey.X = K.ex.Y;
			K.ey.Y = mA + mB + iA * m_rA.X * m_rA.X + iB * m_rB.X * m_rB.X;

			m_linearMass = K.GetInverse();

			m_angularMass = iA + iB;
			if (m_angularMass > 0.0f)
			{
				m_angularMass = 1.0f / m_angularMass;
			}

			if (data.step.warmStarting)
			{
				// Scale impulses to support a variable time step.
				m_linearImpulse *= data.step.dtRatio;
				m_angularImpulse *= data.step.dtRatio;

				Vec2 P = new Vec2(m_linearImpulse.X, m_linearImpulse.Y);
				vA -= mA * P;
				wA -= iA * (Utilities.Cross(m_rA, P) + m_angularImpulse);
				vB += mB * P;
				wB += iB * (Utilities.Cross(m_rB, P) + m_angularImpulse);
			}
			else
			{
				m_linearImpulse.SetZero();
				m_angularImpulse = 0.0f;
			}

			var VelA = data.velocities[m_indexA];
			VelA.v = vA;
			VelA.w = wA;
			data.velocities[m_indexA] = VelA;

			var VelB = data.velocities[m_indexB];
			VelB.v = vB;
			VelB.w = wB;
			data.velocities[m_indexB] = VelB;
		}

		internal override void SolveVelocityConstraints(SolverData data) {
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			float h = data.step.dt;

			// Solve angular friction
			{
				float Cdot = wB - wA;
				float impulse = -m_angularMass * Cdot;

				float oldImpulse = m_angularImpulse;
				float maxImpulse = h * m_maxTorque;
				m_angularImpulse = Math.Max(-maxImpulse, Math.Min(m_angularImpulse + impulse, maxImpulse));
				impulse = m_angularImpulse - oldImpulse;

				wA -= iA * impulse;
				wB += iB * impulse;
			}

			// Solve linear friction
			{
				Vec2 Cdot = vB + Utilities.Cross(wB, m_rB) - vA - Utilities.Cross(wA, m_rA);

				Vec2 impulse = -Utilities.Mul(m_linearMass, Cdot);
				Vec2 oldImpulse = m_linearImpulse;
				m_linearImpulse += impulse;

				float maxImpulse = h * m_maxForce;

				if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
					m_linearImpulse.Normalize();
					m_linearImpulse *= maxImpulse;
				}

				impulse = m_linearImpulse - oldImpulse;

				vA -= mA * impulse;
				wA -= iA * Utilities.Cross(m_rA, impulse);

				vB += mB * impulse;
				wB += iB * Utilities.Cross(m_rB, impulse);
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}

		internal override bool SolvePositionConstraints(SolverData data) {
			return true;
		}

		protected Vec2 m_localAnchorA;
		protected Vec2 m_localAnchorB;

		// Solver shared
		protected Vec2 m_linearImpulse;
		protected float m_angularImpulse;
		protected float m_maxForce;
		protected float m_maxTorque;

		// Solver temp
		protected int m_indexA;
		protected int m_indexB;
		protected Vec2 m_rA;
		protected Vec2 m_rB;
		protected Vec2 m_localCenterA;
		protected Vec2 m_localCenterB;
		protected float m_invMassA;
		protected float m_invMassB;
		protected float m_invIA;
		protected float m_invIB;
		protected Mat22 m_linearMass;
		protected float m_angularMass;
	}
}
