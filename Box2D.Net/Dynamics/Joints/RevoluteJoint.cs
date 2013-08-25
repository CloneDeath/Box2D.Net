using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A revolute joint constrains two bodies to share a common point while they
	/// are free to rotate about the point. The relative rotation about the shared
	/// point is the joint angle. You can limit the relative rotation with
	/// a joint limit that specifies a lower and upper angle. You can use a motor
	/// to drive the relative rotation about the shared point. A maximum motor torque
	/// is provided so that infinite forces are not generated.
	public class RevoluteJoint : Joint
	{
	
		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// Get the reference angle.
		public float GetReferenceAngle() { return m_referenceAngle; }

		/// Get the current joint angle in radians.
		public float GetJointAngle(){
			Body bA = m_bodyA;
			Body bB = m_bodyB;
			return bB.m_sweep.a - bA.m_sweep.a - m_referenceAngle;
		}

		/// Get the current joint angle speed in radians per second.
		public float GetJointSpeed(){
			Body bA = m_bodyA;
			Body bB = m_bodyB;
			return bB.m_angularVelocity - bA.m_angularVelocity;
		}

		/// Is the joint limit enabled?
		public bool IsLimitEnabled() {
			return m_enableLimit;
		}

		/// Enable/disable the joint limit.
		public void EnableLimit(bool flag){
			if (flag != m_enableLimit)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_enableLimit = flag;
				m_impulse.Z = 0.0f;
			}
		}

		/// Get the lower joint limit in radians.
		public float GetLowerLimit(){
			return m_lowerAngle;
		}

		/// Get the upper joint limit in radians.
		public float GetUpperLimit(){
			return m_upperAngle;
		}

		/// Set the joint limits in radians.
		public void SetLimits(float lower, float upper){
			Utilities.Assert(lower <= upper);
	
			if (lower != m_lowerAngle || upper != m_upperAngle)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_impulse.Z = 0.0f;
				m_lowerAngle = lower;
				m_upperAngle = upper;
			}
		}

		/// Is the joint motor enabled?
		public bool IsMotorEnabled() {
			return m_enableMotor;
		}

		/// Enable/disable the joint motor.
		public void EnableMotor(bool flag){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_enableMotor = flag;
		}

		/// Set the motor speed in radians per second.
		public void SetMotorSpeed(float speed){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_motorSpeed = speed;
		}

		/// Get the motor speed in radians per second.
		public float GetMotorSpeed(){
			return m_motorSpeed;
		}

		/// Set the maximum motor torque, usually in N-m.
		public void SetMaxMotorTorque(float torque){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_maxMotorTorque = torque;
		}
		public float GetMaxMotorTorque() { return m_maxMotorTorque; }

		/// Get the reaction force given the inverse time step.
		/// Unit is N.
		public override Vec2 GetReactionForce(float inv_dt){
			Vec2 P = new Vec2(m_impulse.X, m_impulse.Y);
			return inv_dt * P;
		}

		/// Get the reaction torque due to the joint limit given the inverse time step.
		/// Unit is N*m.
		public override float GetReactionTorque(float inv_dt){
			return inv_dt * m_impulse.Z;
		}

		/// Get the current motor torque given the inverse time step.
		/// Unit is N*m.
		public float GetMotorTorque(float inv_dt){
			return inv_dt * m_motorImpulse;
		}

		/// Dump to Settings.Log.
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  RevoluteJointDef jd = new RevoluteJointDef();\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.X, m_localAnchorA.Y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.X, m_localAnchorB.Y);
			Settings.Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
			Settings.Log("  jd.enableLimit = (bool)(%d);\n", m_enableLimit);
			Settings.Log("  jd.lowerAngle = %.15lef;\n", m_lowerAngle);
			Settings.Log("  jd.upperAngle = %.15lef;\n", m_upperAngle);
			Settings.Log("  jd.enableMotor = (bool)(%d);\n", m_enableMotor);
			Settings.Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
			Settings.Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}

		internal RevoluteJoint(RevoluteJointDef def): base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_referenceAngle = def.referenceAngle;

			m_impulse.SetZero();
			m_motorImpulse = 0.0f;

			m_lowerAngle = def.lowerAngle;
			m_upperAngle = def.upperAngle;
			m_maxMotorTorque = def.maxMotorTorque;
			m_motorSpeed = def.motorSpeed;
			m_enableLimit = def.enableLimit;
			m_enableMotor = def.enableMotor;
			m_limitState = LimitState.e_inactiveLimit;
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

			bool fixedRotation = (iA + iB == 0.0f);

			m_mass.ex.X = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
			m_mass.ey.X = -m_rA.Y * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
			m_mass.ez.X = -m_rA.Y * iA - m_rB.Y * iB;
			m_mass.ex.Y = m_mass.ey.X;
			m_mass.ey.Y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
			m_mass.ez.Y = m_rA.X * iA + m_rB.X * iB;
			m_mass.ex.Z = m_mass.ez.X;
			m_mass.ey.Z = m_mass.ez.Y;
			m_mass.ez.Z = iA + iB;

			m_motorMass = iA + iB;
			if (m_motorMass > 0.0f)
			{
				m_motorMass = 1.0f / m_motorMass;
			}

			if (m_enableMotor == false || fixedRotation)
			{
				m_motorImpulse = 0.0f;
			}

			if (m_enableLimit && fixedRotation == false)
			{
				float jointAngle = aB - aA - m_referenceAngle;
				if (Math.Abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings._angularSlop)
				{
					m_limitState = LimitState.e_equalLimits;
				}
				else if (jointAngle <= m_lowerAngle)
				{
					if (m_limitState != LimitState.e_atLowerLimit)
					{
						m_impulse.Z = 0.0f;
					}
					m_limitState = LimitState.e_atLowerLimit;
				}
				else if (jointAngle >= m_upperAngle)
				{
					if (m_limitState != LimitState.e_atUpperLimit)
					{
						m_impulse.Z = 0.0f;
					}
					m_limitState = LimitState.e_atUpperLimit;
				}
				else
				{
					m_limitState = LimitState.e_inactiveLimit;
					m_impulse.Z = 0.0f;
				}
			}
			else
			{
				m_limitState = LimitState.e_inactiveLimit;
			}

			if (data.step.warmStarting)
			{
				// Scale impulses to support a variable time step.
				m_impulse *= data.step.dtRatio;
				m_motorImpulse *= data.step.dtRatio;

				Vec2 P = new Vec2(m_impulse.X, m_impulse.Y);

				vA -= mA * P;
				wA -= iA * (Utilities.Cross(m_rA, P) + m_motorImpulse + m_impulse.Z);

				vB += mB * P;
				wB += iB * (Utilities.Cross(m_rB, P) + m_motorImpulse + m_impulse.Z);
			}
			else
			{
				m_impulse.SetZero();
				m_motorImpulse = 0.0f;
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

			bool fixedRotation = (iA + iB == 0.0f);

			// Solve motor constraint.
			if (m_enableMotor && m_limitState !=LimitState.e_equalLimits && fixedRotation == false)
			{
				float Cdot = wB - wA - m_motorSpeed;
				float impulse = -m_motorMass * Cdot;
				float oldImpulse = m_motorImpulse;
				float maxImpulse = data.step.dt * m_maxMotorTorque;
				m_motorImpulse = Utilities.Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = m_motorImpulse - oldImpulse;

				wA -= iA * impulse;
				wB += iB * impulse;
			}

			// Solve limit constraint.
			if (m_enableLimit && m_limitState != LimitState.e_inactiveLimit && fixedRotation == false)
			{
				Vec2 Cdot1 = vB + Utilities.Cross(wB, m_rB) - vA - Utilities.Cross(wA, m_rA);
				float Cdot2 = wB - wA;
				Vec3 Cdot = new Vec3(Cdot1.X, Cdot1.Y, Cdot2);

				Vec3 impulse = -m_mass.Solve33(Cdot);

				if (m_limitState ==LimitState.e_equalLimits)
				{
					m_impulse += impulse;
				}
				else if (m_limitState == LimitState.e_atLowerLimit)
				{
					float newImpulse = m_impulse.Z + impulse.Z;
					if (newImpulse < 0.0f)
					{
						Vec2 rhs = -Cdot1 + m_impulse.Z * new Vec2(m_mass.ez.X, m_mass.ez.Y);
						Vec2 reduced = m_mass.Solve22(rhs);
						impulse.X = reduced.X;
						impulse.Y = reduced.Y;
						impulse.Z = -m_impulse.Z;
						m_impulse.X += reduced.X;
						m_impulse.Y += reduced.Y;
						m_impulse.Z = 0.0f;
					}
					else
					{
						m_impulse += impulse;
					}
				}
				else if (m_limitState == LimitState.e_atUpperLimit)
				{
					float newImpulse = m_impulse.Z + impulse.Z;
					if (newImpulse > 0.0f)
					{
						Vec2 rhs = -Cdot1 + m_impulse.Z * new Vec2(m_mass.ez.X, m_mass.ez.Y);
						Vec2 reduced = m_mass.Solve22(rhs);
						impulse.X = reduced.X;
						impulse.Y = reduced.Y;
						impulse.Z = -m_impulse.Z;
						m_impulse.X += reduced.X;
						m_impulse.Y += reduced.Y;
						m_impulse.Z = 0.0f;
					}
					else
					{
						m_impulse += impulse;
					}
				}

				Vec2 P = new Vec2(impulse.X, impulse.Y);

				vA -= mA * P;
				wA -= iA * (Utilities.Cross(m_rA, P) + impulse.Z);

				vB += mB * P;
				wB += iB * (Utilities.Cross(m_rB, P) + impulse.Z);
			}
			else
			{
				// Solve point-to-point constraint
				Vec2 Cdot = vB + Utilities.Cross(wB, m_rB) - vA - Utilities.Cross(wA, m_rA);
				Vec2 impulse = m_mass.Solve22(-Cdot);

				m_impulse.X += impulse.X;
				m_impulse.Y += impulse.Y;

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
		internal override bool SolvePositionConstraints(SolverData data){
			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			Rot qA = new Rot(aA);
			Rot qB = new Rot(aB);

			float angularError = 0.0f;
			float positionError = 0.0f;

			bool fixedRotation = (m_invIA + m_invIB == 0.0f);

			// Solve angular limit constraint.
			if (m_enableLimit && m_limitState != LimitState.e_inactiveLimit && fixedRotation == false)
			{
				float angle = aB - aA - m_referenceAngle;
				float limitImpulse = 0.0f;

				if (m_limitState ==LimitState.e_equalLimits)
				{
					// Prevent large angular corrections
					float C = Utilities.Clamp(angle - m_lowerAngle, -Settings._maxAngularCorrection, Settings._maxAngularCorrection);
					limitImpulse = -m_motorMass * C;
					angularError = Math.Abs(C);
				}
				else if (m_limitState == LimitState.e_atLowerLimit)
				{
					float C = angle - m_lowerAngle;
					angularError = -C;

					// Prevent large angular corrections and allow some slop.
					C = Utilities.Clamp(C + Settings._angularSlop, -Settings._maxAngularCorrection, 0.0f);
					limitImpulse = -m_motorMass * C;
				}
				else if (m_limitState == LimitState.e_atUpperLimit)
				{
					float C = angle - m_upperAngle;
					angularError = C;

					// Prevent large angular corrections and allow some slop.
					C = Utilities.Clamp(C - Settings._angularSlop, 0.0f, Settings._maxAngularCorrection);
					limitImpulse = -m_motorMass * C;
				}

				aA -= m_invIA * limitImpulse;
				aB += m_invIB * limitImpulse;
			}

			// Solve point-to-point constraint.
			{
				qA.Set(aA);
				qB.Set(aB);
				Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
				Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);

				Vec2 C = cB + rB - cA - rA;
				positionError = C.Length();

				float mA = m_invMassA, mB = m_invMassB;
				float iA = m_invIA, iB = m_invIB;

				Mat22 K;
				K.ex.X = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
				K.ex.Y = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
				K.ey.X = K.ex.Y;
				K.ey.Y = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;

				Vec2 impulse = -K.Solve(C);

				cA -= mA * impulse;
				aA -= iA * Utilities.Cross(rA, impulse);

				cB += mB * impulse;
				aB += iB * Utilities.Cross(rB, impulse);
			}

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;
	
			return positionError <=Settings._linearSlop && angularError <= Settings._angularSlop;
		}

		// Solver shared
		public Vec2 m_localAnchorA;
		public Vec2 m_localAnchorB;
		public Vec3 m_impulse;
		public float m_motorImpulse;

		public bool m_enableMotor;
		public float m_maxMotorTorque;
		public float m_motorSpeed;

		public bool m_enableLimit;
		public float m_referenceAngle;
		public float m_lowerAngle;
		public float m_upperAngle;

		// Solver temp
		public int m_indexA;
		public int m_indexB;
		public Vec2 m_rA;
		public Vec2 m_rB;
		public Vec2 m_localCenterA;
		public Vec2 m_localCenterB;
		public float m_invMassA;
		public float m_invMassB;
		public float m_invIA;
		public float m_invIB;
		public Mat33 m_mass;			// effective mass for point-to-point constraint.
		public float m_motorMass;	// effective mass for motor/limit angular constraint.
		public LimitState m_limitState;
	};
}
