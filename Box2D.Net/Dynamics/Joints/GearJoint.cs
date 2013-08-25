using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A gear joint is used to connect two joints together. Either joint
	/// can be a revolute or prismatic joint. You specify a gear ratio
	/// to bind the motions together:
	/// coordinate1 + ratio * coordinate2 = constant
	/// The ratio can be negative or positive. If one joint is a revolute joint
	/// and the other joint is a prismatic joint, then the ratio will have units
	/// of length or units of 1/length.
	/// @warning You have to manually destroy the gear joint if joint1 or joint2
	/// is destroyed.

	// Gear Joint:
	// C0 = (coordinate1 + ratio * coordinate2)_initial
	// C = (coordinate1 + ratio * coordinate2) - C0 = 0
	// J = [J1 ratio * J2]
	// K = J * invM * JT
	//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
	//
	// Revolute:
	// coordinate = rotation
	// Cdot = angularVelocity
	// J = [0 0 1]
	// K = J * invM * JT = invI
	//
	// Prismatic:
	// coordinate = dot(p - pg, ug)
	// Cdot = dot(v + cross(w, r), ug)
	// J = [ug cross(r, ug)]
	// K = J * invM * JT = invMass + invI * cross(r, ug)^2
	public class GearJoint : Joint
	{
	
		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override Vec2 GetReactionForce(float inv_dt){
			Vec2 P = m_impulse * m_JvAC;
			return inv_dt * P;
		}

		public override float GetReactionTorque(float inv_dt){
			float L = m_impulse * m_JwA;
			return inv_dt * L;
		}

		/// Get the first joint.
		public Joint GetJoint1() { return m_joint1; }

		/// Get the second joint.
		public Joint GetJoint2() { return m_joint2; }

		/// Set/Get the gear ratio.
		public void SetRatio(float ratio){
			Utilities.Assert(Utilities.IsValid(ratio));
			m_ratio = ratio;
		}
		public float GetRatio(){
			return m_ratio;
		}

		/// Dump joint to dmLog
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			int index1 = m_joint1.m_index;
			int index2 = m_joint2.m_index;

			Settings.Log("  GearJointDef jd;\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.joint1 = joints[%d];\n", index1);
			Settings.Log("  jd.joint2 = joints[%d];\n", index2);
			Settings.Log("  jd.ratio = %.15lef;\n", m_ratio);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}

		
		internal GearJoint(GearJointDef def): base(def)
		{
			m_joint1 = def.joint1;
			m_joint2 = def.joint2;

			m_typeA = m_joint1.GetJointType();
			m_typeB = m_joint2.GetJointType();

			Utilities.Assert(m_typeA == JointType.e_revoluteJoint || m_typeA == JointType.e_prismaticJoint);
			Utilities.Assert(m_typeB == JointType.e_revoluteJoint || m_typeB == JointType.e_prismaticJoint);

			float coordinateA, coordinateB;

			// TODO_ERIN there might be some problem with the joint edges in Joint.

			m_bodyC = m_joint1.GetBodyA();
			m_bodyA = m_joint1.GetBodyB();

			// Get geometry of joint1
			Transform xfA = m_bodyA.m_xf;
			float aA = m_bodyA.m_sweep.a;
			Transform xfC = m_bodyC.m_xf;
			float aC = m_bodyC.m_sweep.a;

			if (m_typeA == JointType.e_revoluteJoint)
			{
				RevoluteJoint revolute = (RevoluteJoint)def.joint1;
				m_localAnchorC = revolute.m_localAnchorA;
				m_localAnchorA = revolute.m_localAnchorB;
				m_referenceAngleA = revolute.m_referenceAngle;
				m_localAxisC.SetZero();

				coordinateA = aA - aC - m_referenceAngleA;
			}
			else
			{
				PrismaticJoint prismatic = (PrismaticJoint)def.joint1;
				m_localAnchorC = prismatic.m_localAnchorA;
				m_localAnchorA = prismatic.m_localAnchorB;
				m_referenceAngleA = prismatic.m_referenceAngle;
				m_localAxisC = prismatic.m_localXAxisA;

				Vec2 pC = m_localAnchorC;
				Vec2 pA = Utilities.MulT(xfC.q, Utilities.Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
				coordinateA = Utilities.Dot(pA - pC, m_localAxisC);
			}

			m_bodyD = m_joint2.GetBodyA();
			m_bodyB = m_joint2.GetBodyB();

			// Get geometry of joint2
			Transform xfB = m_bodyB.m_xf;
			float aB = m_bodyB.m_sweep.a;
			Transform xfD = m_bodyD.m_xf;
			float aD = m_bodyD.m_sweep.a;

			if (m_typeB == JointType.e_revoluteJoint)
			{
				RevoluteJoint revolute = (RevoluteJoint)def.joint2;
				m_localAnchorD = revolute.m_localAnchorA;
				m_localAnchorB = revolute.m_localAnchorB;
				m_referenceAngleB = revolute.m_referenceAngle;
				m_localAxisD.SetZero();

				coordinateB = aB - aD - m_referenceAngleB;
			}
			else
			{
				PrismaticJoint prismatic = (PrismaticJoint)def.joint2;
				m_localAnchorD = prismatic.m_localAnchorA;
				m_localAnchorB = prismatic.m_localAnchorB;
				m_referenceAngleB = prismatic.m_referenceAngle;
				m_localAxisD = prismatic.m_localXAxisA;

				Vec2 pD = m_localAnchorD;
				Vec2 pB = Utilities.MulT(xfD.q, Utilities.Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
				coordinateB = Utilities.Dot(pB - pD, m_localAxisD);
			}

			m_ratio = def.ratio;

			m_constant = coordinateA + m_ratio * coordinateB;

			m_impulse = 0.0f;
		}

		internal override void InitVelocityConstraints(SolverData data){
			m_indexA = m_bodyA.m_islandIndex;
			m_indexB = m_bodyB.m_islandIndex;
			m_indexC = m_bodyC.m_islandIndex;
			m_indexD = m_bodyD.m_islandIndex;
			m_lcA = m_bodyA.m_sweep.localCenter;
			m_lcB = m_bodyB.m_sweep.localCenter;
			m_lcC = m_bodyC.m_sweep.localCenter;
			m_lcD = m_bodyD.m_sweep.localCenter;
			m_mA = m_bodyA.m_invMass;
			m_mB = m_bodyB.m_invMass;
			m_mC = m_bodyC.m_invMass;
			m_mD = m_bodyD.m_invMass;
			m_iA = m_bodyA.m_invI;
			m_iB = m_bodyB.m_invI;
			m_iC = m_bodyC.m_invI;
			m_iD = m_bodyD.m_invI;

			float aA = data.positions[m_indexA].a;
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			float aB = data.positions[m_indexB].a;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			float aC = data.positions[m_indexC].a;
			Vec2 vC = data.velocities[m_indexC].v;
			float wC = data.velocities[m_indexC].w;

			float aD = data.positions[m_indexD].a;
			Vec2 vD = data.velocities[m_indexD].v;
			float wD = data.velocities[m_indexD].w;

			Rot qA = new Rot(aA);
			Rot qB = new Rot(aB);
			Rot qC= new Rot(aC);
			Rot qD= new Rot(aD);

			m_mass = 0.0f;

			if (m_typeA == JointType.e_revoluteJoint)
			{
				m_JvAC.SetZero();
				m_JwA = 1.0f;
				m_JwC = 1.0f;
				m_mass += m_iA + m_iC;
			}
			else
			{
				Vec2 u = Utilities.Mul(qC, m_localAxisC);
				Vec2 rC = Utilities.Mul(qC, m_localAnchorC - m_lcC);
				Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_lcA);
				m_JvAC = u;
				m_JwC = Utilities.Cross(rC, u);
				m_JwA = Utilities.Cross(rA, u);
				m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
			}

			if (m_typeB == JointType.e_revoluteJoint)
			{
				m_JvBD.SetZero();
				m_JwB = m_ratio;
				m_JwD = m_ratio;
				m_mass += m_ratio * m_ratio * (m_iB + m_iD);
			}
			else
			{
				Vec2 u = Utilities.Mul(qD, m_localAxisD);
				Vec2 rD = Utilities.Mul(qD, m_localAnchorD - m_lcD);
				Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_lcB);
				m_JvBD = m_ratio * u;
				m_JwD = m_ratio * Utilities.Cross(rD, u);
				m_JwB = m_ratio * Utilities.Cross(rB, u);
				m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
			}

			// Compute effective mass.
			m_mass = m_mass > 0.0f ? 1.0f / m_mass : 0.0f;

			if (data.step.warmStarting)
			{
				vA += (m_mA * m_impulse) * m_JvAC;
				wA += m_iA * m_impulse * m_JwA;
				vB += (m_mB * m_impulse) * m_JvBD;
				wB += m_iB * m_impulse * m_JwB;
				vC -= (m_mC * m_impulse) * m_JvAC;
				wC -= m_iC * m_impulse * m_JwC;
				vD -= (m_mD * m_impulse) * m_JvBD;
				wD -= m_iD * m_impulse * m_JwD;
			}
			else
			{
				m_impulse = 0.0f;
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
			data.velocities[m_indexC].v = vC;
			data.velocities[m_indexC].w = wC;
			data.velocities[m_indexD].v = vD;
			data.velocities[m_indexD].w = wD;
		}

		internal override void SolveVelocityConstraints(SolverData data){
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;
			Vec2 vC = data.velocities[m_indexC].v;
			float wC = data.velocities[m_indexC].w;
			Vec2 vD = data.velocities[m_indexD].v;
			float wD = data.velocities[m_indexD].w;

			float Cdot = Utilities.Dot(m_JvAC, vA - vC) + Utilities.Dot(m_JvBD, vB - vD);
			Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);

			float impulse = -m_mass * Cdot;
			m_impulse += impulse;

			vA += (m_mA * impulse) * m_JvAC;
			wA += m_iA * impulse * m_JwA;
			vB += (m_mB * impulse) * m_JvBD;
			wB += m_iB * impulse * m_JwB;
			vC -= (m_mC * impulse) * m_JvAC;
			wC -= m_iC * impulse * m_JwC;
			vD -= (m_mD * impulse) * m_JvBD;
			wD -= m_iD * impulse * m_JwD;

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
			data.velocities[m_indexC].v = vC;
			data.velocities[m_indexC].w = wC;
			data.velocities[m_indexD].v = vD;
			data.velocities[m_indexD].w = wD;
		}

		internal override bool SolvePositionConstraints(SolverData data){
			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			Vec2 cC = data.positions[m_indexC].c;
			float aC = data.positions[m_indexC].a;
			Vec2 cD = data.positions[m_indexD].c;
			float aD = data.positions[m_indexD].a;

			Rot qA = new Rot(aA);
			Rot qB = new Rot(aB);
			Rot qC= new Rot(aC);
			Rot qD = new Rot(aD);

			float linearError = 0.0f;

			float coordinateA, coordinateB;

			Vec2 JvAC = new Vec2();
			Vec2 JvBD = new Vec2();
			float JwA, JwB, JwC, JwD;
			float mass = 0.0f;

			if (m_typeA == JointType.e_revoluteJoint)
			{
				JvAC.SetZero();
				JwA = 1.0f;
				JwC = 1.0f;
				mass += m_iA + m_iC;

				coordinateA = aA - aC - m_referenceAngleA;
			}
			else
			{
				Vec2 u = Utilities.Mul(qC, m_localAxisC);
				Vec2 rC = Utilities.Mul(qC, m_localAnchorC - m_lcC);
				Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_lcA);
				JvAC = u;
				JwC = Utilities.Cross(rC, u);
				JwA = Utilities.Cross(rA, u);
				mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

				Vec2 pC = m_localAnchorC - m_lcC;
				Vec2 pA = Utilities.MulT(qC, rA + (cA - cC));
				coordinateA = Utilities.Dot(pA - pC, m_localAxisC);
			}

			if (m_typeB == JointType.e_revoluteJoint)
			{
				JvBD.SetZero();
				JwB = m_ratio;
				JwD = m_ratio;
				mass += m_ratio * m_ratio * (m_iB + m_iD);

				coordinateB = aB - aD - m_referenceAngleB;
			}
			else
			{
				Vec2 u = Utilities.Mul(qD, m_localAxisD);
				Vec2 rD = Utilities.Mul(qD, m_localAnchorD - m_lcD);
				Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_lcB);
				JvBD = m_ratio * u;
				JwD = m_ratio * Utilities.Cross(rD, u);
				JwB = m_ratio * Utilities.Cross(rB, u);
				mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;

				Vec2 pD = m_localAnchorD - m_lcD;
				Vec2 pB = Utilities.MulT(qD, rB + (cB - cD));
				coordinateB = Utilities.Dot(pB - pD, m_localAxisD);
			}

			float C = (coordinateA + m_ratio * coordinateB) - m_constant;

			float impulse = 0.0f;
			if (mass > 0.0f)
			{
				impulse = -C / mass;
			}

			cA += m_mA * impulse * JvAC;
			aA += m_iA * impulse * JwA;
			cB += m_mB * impulse * JvBD;
			aB += m_iB * impulse * JwB;
			cC -= m_mC * impulse * JvAC;
			aC -= m_iC * impulse * JwC;
			cD -= m_mD * impulse * JvBD;
			aD -= m_iD * impulse * JwD;

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;
			data.positions[m_indexC].c = cC;
			data.positions[m_indexC].a = aC;
			data.positions[m_indexD].c = cD;
			data.positions[m_indexD].a = aD;

			// TODO_ERIN not implemented
			return linearError <Settings._linearSlop;
		}

		Joint m_joint1;
		Joint m_joint2;

		JointType m_typeA;
		JointType m_typeB;

		// Body A is connected to body C
		// Body B is connected to body D
		Body m_bodyC;
		Body m_bodyD;

		// Solver shared
		Vec2 m_localAnchorA;
		Vec2 m_localAnchorB;
		Vec2 m_localAnchorC;
		Vec2 m_localAnchorD;

		Vec2 m_localAxisC;
		Vec2 m_localAxisD;

		float m_referenceAngleA;
		float m_referenceAngleB;

		float m_constant;
		float m_ratio;

		float m_impulse;

		// Solver temp
		int m_indexA, m_indexB, m_indexC, m_indexD;
		Vec2 m_lcA, m_lcB, m_lcC, m_lcD;
		float m_mA, m_mB, m_mC, m_mD;
		float m_iA, m_iB, m_iC, m_iD;
		Vec2 m_JvAC, m_JvBD;
		float m_JwA, m_JwB, m_JwC, m_JwD;
		float m_mass;
	};
}
