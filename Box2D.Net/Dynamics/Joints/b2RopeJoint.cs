using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D.Dynamics.Joints {
	/// A rope joint enforces a maximum distance between two points
	/// on two bodies. It has no other effect.
	/// Warning: if you attempt to change the maximum length during
	/// the simulation you will get some non-physical behavior.
	/// A model that would allow you to dynamically modify the length
	/// would have some sponginess, so I chose not to implement it
	/// that way. See DistanceJoint if you want to dynamically
	/// control length.
	public class RopeJoint : Joint
	{
	
		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override Vec2 GetReactionForce(float inv_dt){
			Vec2 F = (inv_dt * m_impulse) * m_u;
			return F;
		}
		public override float GetReactionTorque(float inv_dt){
			return 0.0f;
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// Set/Get the maximum length of the rope.
		public void SetMaxLength(float length) { m_maxLength = length; }
		public float GetMaxLength(){
			return m_maxLength;
		}

		public LimitState GetLimitState(){
			return m_state;
		}

		/// Dump joint to dmLog
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  RopeJointDef jd;\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
			Settings.Log("  jd.maxLength = %.15lef;\n", m_maxLength);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}
		
		// Limit:
		// C = norm(pB - pA) - L
		// u = (pB - pA) / norm(pB - pA)
		// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
		// J = [-u -cross(rA, u) u cross(rB, u)]
		// K = J * invM * JT
		//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2
		internal RopeJoint(RopeJointDef def): base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;

			m_maxLength = def.maxLength;

			m_mass = 0.0f;
			m_impulse = 0.0f;
			m_state = LimitState.e_inactiveLimit;
			m_length = 0.0f;
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
			m_u = cB + m_rB - cA - m_rA;

			m_length = m_u.Length();

			float C = m_length - m_maxLength;
			if (C > 0.0f)
			{
				m_state = LimitState.e_atUpperLimit;
			}
			else
			{
				m_state = LimitState.e_inactiveLimit;
			}

			if (m_length >Settings._linearSlop)
			{
				m_u *= 1.0f / m_length;
			}
			else
			{
				m_u.SetZero();
				m_mass = 0.0f;
				m_impulse = 0.0f;
				return;
			}

			// Compute effective mass.
			float crA = Utilities.Cross(m_rA, m_u);
			float crB = Utilities.Cross(m_rB, m_u);
			float invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

			m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

			if (data.step.warmStarting)
			{
				// Scale the impulse to support a variable time step.
				m_impulse *= data.step.dtRatio;

				Vec2 P = m_impulse * m_u;
				vA -= m_invMassA * P;
				wA -= m_invIA * Utilities.Cross(m_rA, P);
				vB += m_invMassB * P;
				wB += m_invIB * Utilities.Cross(m_rB, P);
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

			// Cdot = dot(u, v + cross(w, r))
			Vec2 vpA = vA + Utilities.Cross(wA, m_rA);
			Vec2 vpB = vB + Utilities.Cross(wB, m_rB);
			float C = m_length - m_maxLength;
			float Cdot = Utilities.Dot(m_u, vpB - vpA);

			// Predictive constraint.
			if (C < 0.0f)
			{
				Cdot += data.step.inv_dt * C;
			}

			float impulse = -m_mass * Cdot;
			float oldImpulse = m_impulse;
			m_impulse = Math.Min(0.0f, m_impulse + impulse);
			impulse = m_impulse - oldImpulse;

			Vec2 P = impulse * m_u;
			vA -= m_invMassA * P;
			wA -= m_invIA * Utilities.Cross(m_rA, P);
			vB += m_invMassB * P;
			wB += m_invIB * Utilities.Cross(m_rB, P);

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

			Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);
			Vec2 u = cB + rB - cA - rA;

			float length = u.Normalize();
			float C = length - m_maxLength;

			C = Utilities.Clamp(C, 0.0f, Settings._maxLinearCorrection);

			float impulse = -m_mass * C;
			Vec2 P = impulse * u;

			cA -= m_invMassA * P;
			aA -= m_invIA * Utilities.Cross(rA, P);
			cB += m_invMassB * P;
			aB += m_invIB * Utilities.Cross(rB, P);

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return length - m_maxLength <Settings._linearSlop;
		}

		// Solver shared
		Vec2 m_localAnchorA;
		Vec2 m_localAnchorB;
		float m_maxLength;
		float m_length;
		float m_impulse;

		// Solver temp
		int m_indexA;
		int m_indexB;
		Vec2 m_u;
		Vec2 m_rA;
		Vec2 m_rB;
		Vec2 m_localCenterA;
		Vec2 m_localCenterB;
		float m_invMassA;
		float m_invMassB;
		float m_invIA;
		float m_invIB;
		float m_mass;
		LimitState m_state;
	};
}
