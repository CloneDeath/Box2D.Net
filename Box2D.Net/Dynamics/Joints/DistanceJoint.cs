﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A distance joint constrains two points on two bodies
	/// to remain at a fixed distance from each other. You can view
	/// this as a massless, rigid rod.
	class DistanceJoint : Joint
	{
		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		/// Get the reaction force given the inverse time step.
		/// Unit is N.
		public override Vec2 GetReactionForce(float inv_dt){
			Vec2 F = (inv_dt * m_impulse) * m_u;
			return F;
		}

		/// Get the reaction torque given the inverse time step.
		/// Unit is N*m. This is always zero for a distance joint.
		public override float GetReactionTorque(float inv_dt){
			return 0.0f;
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// Set/get the natural length.
		/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
		public void SetLength(float length){
			m_length = length;
		}

		public float GetLength(){
			return m_length;
		}

		/// Set/get frequency in Hz.
		public void SetFrequency(float hz){
			m_frequencyHz = hz;
		}
		public float GetFrequency(){
			return m_frequencyHz;
		}

		/// Set/get damping ratio.
		public void SetDampingRatio(float ratio){
			m_dampingRatio = ratio;
		}
		public float GetDampingRatio(){
			return m_dampingRatio;
		}

		/// Dump joint to dmLog
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  DistanceJointDef jd;\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.X, m_localAnchorA.Y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.X, m_localAnchorB.Y);
			Settings.Log("  jd.length = %.15lef;\n", m_length);
			Settings.Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
			Settings.Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}
		
		internal DistanceJoint(DistanceJointDef def) : base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_length = def.length;
			m_frequencyHz = def.frequencyHz;
			m_dampingRatio = def.dampingRatio;
			m_impulse = 0.0f;
			m_gamma = 0.0f;
			m_bias = 0.0f;
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

			Rot qA = new Rot(aA); 
			Rot qB = new Rot(aB);

			m_rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			m_rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);
			m_u = cB + m_rB - cA - m_rA;

			// Handle singularity.
			float length = m_u.Length();
			if (length >Settings._linearSlop)
			{
				m_u *= 1.0f / length;
			}
			else
			{
				m_u.Set(0.0f, 0.0f);
			}

			float crAu = Utilities.Cross(m_rA, m_u);
			float crBu = Utilities.Cross(m_rB, m_u);
			float invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

			// Compute the effective mass matrix.
			m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

			if (m_frequencyHz > 0.0f)
			{
				float C = length - m_length;

				// Frequency
				float omega = 2.0f * (float)Math.PI * m_frequencyHz;

				// Damping coefficient
				float d = 2.0f * m_mass * m_dampingRatio * omega;

				// Spring stiffness
				float k = m_mass * omega * omega;

				// magic formulas
				float h = data.step.dt;
				m_gamma = h * (d + h * k);
				m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
				m_bias = C * h * k * m_gamma;

				invMass += m_gamma;
				m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
			}
			else
			{
				m_gamma = 0.0f;
				m_bias = 0.0f;
			}

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
			float Cdot = Utilities.Dot(m_u, vpB - vpA);

			float impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
			m_impulse += impulse;

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
			if (m_frequencyHz > 0.0f)
			{
				// There is no position correction for soft distance constraints.
				return true;
			}

			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			Rot qA = new Rot(aA); Rot qB = new Rot(aB);

			Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);
			Vec2 u = cB + rB - cA - rA;

			float length = u.Normalize();
			float C = length - m_length;
			C = Utilities.Clamp(C, -Settings._maxLinearCorrection, Settings._maxLinearCorrection);

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

			return Math.Abs(C) <Settings._linearSlop;
		}

		float m_frequencyHz;
		float m_dampingRatio;
		float m_bias;

		// Solver shared
		Vec2 m_localAnchorA;
		Vec2 m_localAnchorB;
		float m_gamma;
		float m_impulse;
		float m_length;

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
	};
}
