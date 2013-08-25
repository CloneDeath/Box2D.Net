using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// The pulley joint is connected to two bodies and two fixed ground points.
	/// The pulley supports a ratio such that:
	/// length1 + ratio * length2 <= constant
	/// Yes, the force transmitted is scaled by the ratio.
	/// Warning: the pulley joint can get a bit squirrelly by itself. They often
	/// work better when combined with prismatic joints. You should also cover the
	/// the anchor points with static shapes to prevent one side from going to
	/// zero length.
	public class PulleyJoint : Joint
	{
		public const float _minPulleyLength = 2.0f;
	
		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override Vec2 GetReactionForce(float inv_dt){
			Vec2 P = m_impulse * m_uB;
			return inv_dt * P;
		}
		public override float GetReactionTorque(float inv_dt){
			return 0.0f;
		}

		/// Get the first ground anchor.
		public Vec2 GetGroundAnchorA(){
			return m_groundAnchorA;
		}


		/// Get the second ground anchor.
		public Vec2 GetGroundAnchorB(){
			return m_groundAnchorB;
		}

		/// Get the current length of the segment attached to bodyA.
		public float GetLengthA(){
			return m_lengthA;
		}

		/// Get the current length of the segment attached to bodyB.
		public float GetLengthB(){
			return m_lengthB;
		}

		/// Get the pulley ratio.
		public float GetRatio(){
			return m_ratio;
		}


		/// Get the current length of the segment attached to bodyA.
		public float GetCurrentLengthA(){
			Vec2 p = m_bodyA.GetWorldPoint(m_localAnchorA);
			Vec2 s = m_groundAnchorA;
			Vec2 d = p - s;
			return d.Length();
		}

		/// Get the current length of the segment attached to bodyB.
		public float GetCurrentLengthB(){
			Vec2 p = m_bodyB.GetWorldPoint(m_localAnchorB);
			Vec2 s = m_groundAnchorB;
			Vec2 d = p - s;
			return d.Length();
		}

		/// Dump joint to dmLog
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  PulleyJointDef jd;\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.groundAnchorA.Set(%.15lef, %.15lef);\n", m_groundAnchorA.x, m_groundAnchorA.y);
			Settings.Log("  jd.groundAnchorB.Set(%.15lef, %.15lef);\n", m_groundAnchorB.x, m_groundAnchorB.y);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
			Settings.Log("  jd.lengthA = %.15lef;\n", m_lengthA);
			Settings.Log("  jd.lengthB = %.15lef;\n", m_lengthB);
			Settings.Log("  jd.ratio = %.15lef;\n", m_ratio);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}

		/// Implement Joint::ShiftOrigin
		public void ShiftOrigin(Vec2 newOrigin){
			m_groundAnchorA -= newOrigin;
			m_groundAnchorB -= newOrigin;
		}

		internal PulleyJoint(PulleyJointDef def): base(def)
		{
			m_groundAnchorA = def.groundAnchorA;
			m_groundAnchorB = def.groundAnchorB;
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;

			m_lengthA = def.lengthA;
			m_lengthB = def.lengthB;

			Utilities.Assert(def.ratio != 0.0f);
			m_ratio = def.ratio;

			m_constant = def.lengthA + m_ratio * def.lengthB;

			m_impulse = 0.0f;
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

			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			Rot qA = new Rot(aA); Rot qB = new Rot(aB);

			m_rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			m_rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);

			// Get the pulley axes.
			m_uA = cA + m_rA - m_groundAnchorA;
			m_uB = cB + m_rB - m_groundAnchorB;

			float lengthA = m_uA.Length();
			float lengthB = m_uB.Length();

			if (lengthA > 10.0f *Settings._linearSlop)
			{
				m_uA *= 1.0f / lengthA;
			}
			else
			{
				m_uA.SetZero();
			}

			if (lengthB > 10.0f *Settings._linearSlop)
			{
				m_uB *= 1.0f / lengthB;
			}
			else
			{
				m_uB.SetZero();
			}

			// Compute effective mass.
			float ruA = Utilities.Cross(m_rA, m_uA);
			float ruB = Utilities.Cross(m_rB, m_uB);

			float mA = m_invMassA + m_invIA * ruA * ruA;
			float mB = m_invMassB + m_invIB * ruB * ruB;

			m_mass = mA + m_ratio * m_ratio * mB;

			if (m_mass > 0.0f)
			{
				m_mass = 1.0f / m_mass;
			}

			if (data.step.warmStarting)
			{
				// Scale impulses to support variable time steps.
				m_impulse *= data.step.dtRatio;

				// Warm starting.
				Vec2 PA = -(m_impulse) * m_uA;
				Vec2 PB = (-m_ratio * m_impulse) * m_uB;

				vA += m_invMassA * PA;
				wA += m_invIA * Utilities.Cross(m_rA, PA);
				vB += m_invMassB * PB;
				wB += m_invIB * Utilities.Cross(m_rB, PB);
			}
			else
			{
				m_impulse = 0.0f;
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

			Vec2 vpA = vA + Utilities.Cross(wA, m_rA);
			Vec2 vpB = vB + Utilities.Cross(wB, m_rB);

			float Cdot = -Utilities.Dot(m_uA, vpA) - m_ratio * Utilities.Dot(m_uB, vpB);
			float impulse = -m_mass * Cdot;
			m_impulse += impulse;

			Vec2 PA = -impulse * m_uA;
			Vec2 PB = -m_ratio * impulse * m_uB;
			vA += m_invMassA * PA;
			wA += m_invIA * Utilities.Cross(m_rA, PA);
			vB += m_invMassB * PB;
			wB += m_invIB * Utilities.Cross(m_rB, PB);

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

			Rot qA = new Rot(aA); Rot qB = new Rot(aB);

			Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);

			// Get the pulley axes.
			Vec2 uA = cA + rA - m_groundAnchorA;
			Vec2 uB = cB + rB - m_groundAnchorB;

			float lengthA = uA.Length();
			float lengthB = uB.Length();

			if (lengthA > 10.0f *Settings._linearSlop)
			{
				uA *= 1.0f / lengthA;
			}
			else
			{
				uA.SetZero();
			}

			if (lengthB > 10.0f *Settings._linearSlop)
			{
				uB *= 1.0f / lengthB;
			}
			else
			{
				uB.SetZero();
			}

			// Compute effective mass.
			float ruA = Utilities.Cross(rA, uA);
			float ruB = Utilities.Cross(rB, uB);

			float mA = m_invMassA + m_invIA * ruA * ruA;
			float mB = m_invMassB + m_invIB * ruB * ruB;

			float mass = mA + m_ratio * m_ratio * mB;

			if (mass > 0.0f)
			{
				mass = 1.0f / mass;
			}

			float C = m_constant - lengthA - m_ratio * lengthB;
			float linearError = Math.Abs(C);

			float impulse = -mass * C;

			Vec2 PA = -impulse * uA;
			Vec2 PB = -m_ratio * impulse * uB;

			cA += m_invMassA * PA;
			aA += m_invIA * Utilities.Cross(rA, PA);
			cB += m_invMassB * PB;
			aB += m_invIB * Utilities.Cross(rB, PB);

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return linearError <Settings._linearSlop;
		}

		Vec2 m_groundAnchorA;
		Vec2 m_groundAnchorB;
		float m_lengthA;
		float m_lengthB;
	
		// Solver shared
		Vec2 m_localAnchorA;
		Vec2 m_localAnchorB;
		float m_constant;
		float m_ratio;
		float m_impulse;

		// Solver temp
		int m_indexA;
		int m_indexB;
		Vec2 m_uA;
		Vec2 m_uB;
		Vec2 m_rA;
		Vec2 m_rB;
		Vec2 m_localCenterA;
		Vec2 m_localCenterB;
		float m_invMassA;
		float m_invMassB;
		float m_invIA;
		float m_invIB;
		float m_mass;
	};
}
