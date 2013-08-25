using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A prismatic joint. This joint provides one degree of freedom: translation
	/// along an axis fixed in bodyA. Relative rotation is prevented. You can
	/// use a joint limit to restrict the range of motion and a joint motor to
	/// drive the motion or to model joint friction.
	public class PrismaticJoint : Joint
	{
		public PrismaticJoint(PrismaticJointDef def) : base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_localXAxisA = def.localAxisA;
			m_localXAxisA.Normalize();
			m_localYAxisA = Utilities.Cross(1.0f, m_localXAxisA);
			m_referenceAngle = def.referenceAngle;

			m_impulse.SetZero();
			m_motorMass = 0.0f;
			m_motorImpulse = 0.0f;

			m_lowerTranslation = def.lowerTranslation;
			m_upperTranslation = def.upperTranslation;
			m_maxMotorForce = def.maxMotorForce;
			m_motorSpeed = def.motorSpeed;
			m_enableLimit = def.enableLimit;
			m_enableMotor = def.enableMotor;
			m_limitState = LimitState.e_inactiveLimit;

			m_axis.SetZero();
			m_perp.SetZero();
		}

		public override Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override Vec2 GetAnchorB() {
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override Vec2 GetReactionForce(float inv_dt){
			return inv_dt * (m_impulse.X * m_perp + (m_motorImpulse + m_impulse.Z) * m_axis);
		}
		public override float GetReactionTorque(float inv_dt) {
			return inv_dt * m_impulse.Y;
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// The local joint axis relative to bodyA.
		public Vec2 GetLocalAxisA() { return m_localXAxisA; }

		/// Get the reference angle.
		public float GetReferenceAngle() { return m_referenceAngle; }

		/// Get the current joint translation, usually in meters.
		public float GetJointTranslation(){
			Vec2 pA = m_bodyA.GetWorldPoint(m_localAnchorA);
			Vec2 pB = m_bodyB.GetWorldPoint(m_localAnchorB);
			Vec2 d = pB - pA;
			Vec2 axis = m_bodyA.GetWorldVector(m_localXAxisA);

			float translation = Utilities.Dot(d, axis);
			return translation;
		}

		/// Get the current joint translation speed, usually in meters per second.
		public float GetJointSpeed(){
			Body bA = m_bodyA;
			Body bB = m_bodyB;

			Vec2 rA = Utilities.Mul(bA.m_xf.q, m_localAnchorA - bA.m_sweep.localCenter);
			Vec2 rB = Utilities.Mul(bB.m_xf.q, m_localAnchorB - bB.m_sweep.localCenter);
			Vec2 p1 = bA.m_sweep.c + rA;
			Vec2 p2 = bB.m_sweep.c + rB;
			Vec2 d = p2 - p1;
			Vec2 axis = Utilities.Mul(bA.m_xf.q, m_localXAxisA);

			Vec2 vA = bA.m_linearVelocity;
			Vec2 vB = bB.m_linearVelocity;
			float wA = bA.m_angularVelocity;
			float wB = bB.m_angularVelocity;

			float speed = Utilities.Dot(d, Utilities.Cross(wA, axis)) + Utilities.Dot(axis, vB + Utilities.Cross(wB, rB) - vA - Utilities.Cross(wA, rA));
			return speed;
		}

		/// Is the joint limit enabled?
		public bool IsLimitEnabled(){
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

		/// Get the lower joint limit, usually in meters.
		public float GetLowerLimit(){
			return m_lowerTranslation;
		}

		/// Get the upper joint limit, usually in meters.
		public float GetUpperLimit(){
			return m_upperTranslation;
		}

		/// Set the joint limits, usually in meters.
		public void SetLimits(float lower, float upper){
			Utilities.Assert(lower <= upper);
			if (lower != m_lowerTranslation || upper != m_upperTranslation)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_lowerTranslation = lower;
				m_upperTranslation = upper;
				m_impulse.Z = 0.0f;
			}
		}

		/// Is the joint motor enabled?
		public bool IsMotorEnabled(){
			return m_enableMotor;
		}

		/// Enable/disable the joint motor.
		public void EnableMotor(bool flag){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_enableMotor = flag;
		}

		/// Set the motor speed, usually in meters per second.
		public void SetMotorSpeed(float speed){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_motorSpeed = speed;
		}

		/// Get the motor speed, usually in meters per second.
		public float GetMotorSpeed(){
			return m_motorSpeed;
		}

		/// Set the maximum motor force, usually in N.
		public void SetMaxMotorForce(float force){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_maxMotorForce = force;
		}
		public float GetMaxMotorForce() { return m_maxMotorForce; }

		/// Get the current motor force given the inverse time step, usually in N.
		public float GetMotorForce(float inv_dt){
			return inv_dt * m_motorImpulse;
		}

		/// Dump to Settings.Log
		public override void Dump() {
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			Settings.Log("  PrismaticJointDef jd;\n");
			Settings.Log("  jd.bodyA = bodies[%d];\n", indexA);
			Settings.Log("  jd.bodyB = bodies[%d];\n", indexB);
			Settings.Log("  jd.collideConnected = (bool)(%d);\n", m_collideConnected);
			Settings.Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.X, m_localAnchorA.Y);
			Settings.Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.X, m_localAnchorB.Y);
			Settings.Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA.X, m_localXAxisA.Y);
			Settings.Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
			Settings.Log("  jd.enableLimit = (bool)(%d);\n", m_enableLimit);
			Settings.Log("  jd.lowerTranslation = %.15lef;\n", m_lowerTranslation);
			Settings.Log("  jd.upperTranslation = %.15lef;\n", m_upperTranslation);
			Settings.Log("  jd.enableMotor = (bool)(%d);\n", m_enableMotor);
			Settings.Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
			Settings.Log("  jd.maxMotorForce = %.15lef;\n", m_maxMotorForce);
			Settings.Log("  joints[%d] = m_world.CreateJoint(jd);\n", m_index);
		}


		internal override void InitVelocityConstraints(SolverData data) {
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

			// Compute the effective masses.
			Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);
			Vec2 d = (cB - cA) + rB - rA;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			// Compute motor Jacobian and effective mass.
			{
				m_axis = Utilities.Mul(qA, m_localXAxisA);
				m_a1 = Utilities.Cross(d + rA, m_axis);
				m_a2 = Utilities.Cross(rB, m_axis);

				m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
				if (m_motorMass > 0.0f)
				{
					m_motorMass = 1.0f / m_motorMass;
				}
			}

			// Prismatic constraint.
			{
				m_perp = Utilities.Mul(qA, m_localYAxisA);

				m_s1 = Utilities.Cross(d + rA, m_perp);
				m_s2 = Utilities.Cross(rB, m_perp);

				float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
				float k12 = iA * m_s1 + iB * m_s2;
				float k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
				float k22 = iA + iB;
				if (k22 == 0.0f)
				{
					// For bodies with fixed rotation.
					k22 = 1.0f;
				}
				float k23 = iA * m_a1 + iB * m_a2;
				float k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

				m_K.ex.Set(k11, k12, k13);
				m_K.ey.Set(k12, k22, k23);
				m_K.ez.Set(k13, k23, k33);
			}

			// Compute motor and limit terms.
			if (m_enableLimit)
			{
				float jointTranslation = Utilities.Dot(m_axis, d);
				if (Math.Abs(m_upperTranslation - m_lowerTranslation) < 2.0f *Settings._linearSlop)
				{
					m_limitState = LimitState.e_equalLimits;
				}
				else if (jointTranslation <= m_lowerTranslation)
				{
					if (m_limitState != LimitState.e_atLowerLimit)
					{
						m_limitState = LimitState.e_atLowerLimit;
						m_impulse.Z = 0.0f;
					}
				}
				else if (jointTranslation >= m_upperTranslation)
				{
					if (m_limitState != LimitState.e_atUpperLimit)
					{
						m_limitState = LimitState.e_atUpperLimit;
						m_impulse.Z = 0.0f;
					}
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
				m_impulse.Z = 0.0f;
			}

			if (m_enableMotor == false)
			{
				m_motorImpulse = 0.0f;
			}

			if (data.step.warmStarting)
			{
				// Account for variable time step.
				m_impulse *= data.step.dtRatio;
				m_motorImpulse *= data.step.dtRatio;

				Vec2 P = m_impulse.X * m_perp + (m_motorImpulse + m_impulse.Z) * m_axis;
				float LA = m_impulse.X * m_s1 + m_impulse.Y + (m_motorImpulse + m_impulse.Z) * m_a1;
				float LB = m_impulse.X * m_s2 + m_impulse.Y + (m_motorImpulse + m_impulse.Z) * m_a2;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
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

		internal override void SolveVelocityConstraints(SolverData data) {
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			// Solve linear motor constraint.
			if (m_enableMotor && m_limitState != LimitState.e_equalLimits)
			{
				float Cdot = Utilities.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
				float impulse = m_motorMass * (m_motorSpeed - Cdot);
				float oldImpulse = m_motorImpulse;
				float maxImpulse = data.step.dt * m_maxMotorForce;
				m_motorImpulse = Utilities.Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = m_motorImpulse - oldImpulse;

				Vec2 P = impulse * m_axis;
				float LA = impulse * m_a1;
				float LB = impulse * m_a2;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
			}

			Vec2 Cdot1;
			Cdot1.X = Utilities.Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
			Cdot1.Y = wB - wA;

			if (m_enableLimit && m_limitState != LimitState.e_inactiveLimit)
			{
				// Solve prismatic and limit constraint in block form.
				float Cdot2;
				Cdot2 = Utilities.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
				Vec3 Cdot = new Vec3(Cdot1.X, Cdot1.Y, Cdot2);

				Vec3 f1 = m_impulse;
				Vec3 df =  m_K.Solve33(-Cdot);
				m_impulse += df;

				if (m_limitState == LimitState.e_atLowerLimit)
				{
					m_impulse.Z = Math.Max(m_impulse.Z, 0.0f);
				}
				else if (m_limitState == LimitState.e_atUpperLimit)
				{
					m_impulse.Z = Math.Min(m_impulse.Z, 0.0f);
				}

				// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
				Vec2 b = -Cdot1 - (m_impulse.Z - f1.Z) * new Vec2(m_K.ez.X, m_K.ez.Y);
				Vec2 f2r = m_K.Solve22(b) + new Vec2(f1.X, f1.Y);
				m_impulse.X = f2r.X;
				m_impulse.Y = f2r.Y;

				df = m_impulse - f1;

				Vec2 P = df.X * m_perp + df.Z * m_axis;
				float LA = df.X * m_s1 + df.Y + df.Z * m_a1;
				float LB = df.X * m_s2 + df.Y + df.Z * m_a2;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
			}
			else
			{
				// Limit is inactive, just solve the prismatic constraint in block form.
				Vec2 df = m_K.Solve22(-Cdot1);
				m_impulse.X += df.X;
				m_impulse.Y += df.Y;

				Vec2 P = df.X * m_perp;
				float LA = df.X * m_s1 + df.Y;
				float LB = df.X * m_s2 + df.Y;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}

		internal override bool SolvePositionConstraints(SolverData data) {
			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			Rot qA = new Rot(aA);
			Rot qB = new Rot(aB);

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			// Compute fresh Jacobians
			Vec2 rA = Utilities.Mul(qA, m_localAnchorA - m_localCenterA);
			Vec2 rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);
			Vec2 d = cB + rB - cA - rA;

			Vec2 axis = Utilities.Mul(qA, m_localXAxisA);
			float a1 = Utilities.Cross(d + rA, axis);
			float a2 = Utilities.Cross(rB, axis);
			Vec2 perp = Utilities.Mul(qA, m_localYAxisA);

			float s1 = Utilities.Cross(d + rA, perp);
			float s2 = Utilities.Cross(rB, perp);

			Vec3 impulse;
			Vec2 C1;
			C1.X = Utilities.Dot(perp, d);
			C1.Y = aB - aA - m_referenceAngle;

			float linearError = Math.Abs(C1.X);
			float angularError = Math.Abs(C1.Y);

			bool active = false;
			float C2 = 0.0f;
			if (m_enableLimit)
			{
				float translation = Utilities.Dot(axis, d);
				if (Math.Abs(m_upperTranslation - m_lowerTranslation) < 2.0f *Settings._linearSlop)
				{
					// Prevent large angular corrections
					C2 = Utilities.Clamp(translation, -Settings._maxLinearCorrection, Settings._maxLinearCorrection);
					linearError = Math.Max(linearError, Math.Abs(translation));
					active = true;
				}
				else if (translation <= m_lowerTranslation)
				{
					// Prevent large linear corrections and allow some slop.
					C2 = Utilities.Clamp(translation - m_lowerTranslation + Settings._linearSlop, -Settings._maxLinearCorrection, 0.0f);
					linearError = Math.Max(linearError, m_lowerTranslation - translation);
					active = true;
				}
				else if (translation >= m_upperTranslation)
				{
					// Prevent large linear corrections and allow some slop.
					C2 = Utilities.Clamp(translation - m_upperTranslation - Settings._linearSlop, 0.0f, Settings._maxLinearCorrection);
					linearError = Math.Max(linearError, translation - m_upperTranslation);
					active = true;
				}
			}

			if (active)
			{
				float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
				float k12 = iA * s1 + iB * s2;
				float k13 = iA * s1 * a1 + iB * s2 * a2;
				float k22 = iA + iB;
				if (k22 == 0.0f)
				{
					// For fixed rotation
					k22 = 1.0f;
				}
				float k23 = iA * a1 + iB * a2;
				float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

				Mat33 K = new Mat33();
				K.ex.Set(k11, k12, k13);
				K.ey.Set(k12, k22, k23);
				K.ez.Set(k13, k23, k33);

				Vec3 C = new Vec3();
				C.X = C1.X;
				C.Y = C1.Y;
				C.Z = C2;

				impulse = K.Solve33(-C);
			}
			else
			{
				float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
				float k12 = iA * s1 + iB * s2;
				float k22 = iA + iB;
				if (k22 == 0.0f)
				{
					k22 = 1.0f;
				}

				Mat22 K = new Mat22();
				K.ex.Set(k11, k12);
				K.ey.Set(k12, k22);

				Vec2 impulse1 = K.Solve(-C1);
				impulse.X = impulse1.X;
				impulse.Y = impulse1.Y;
				impulse.Z = 0.0f;
			}

			Vec2 P = impulse.X * perp + impulse.Z * axis;
			float LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
			float LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

			cA -= mA * P;
			aA -= iA * LA;
			cB += mB * P;
			aB += iB * LB;

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return linearError <= Settings._linearSlop && angularError <= Settings._angularSlop;
		}

		// Solver shared
		internal Vec2 m_localAnchorA;
		internal Vec2 m_localAnchorB;
		internal Vec2 m_localXAxisA;
		internal Vec2 m_localYAxisA;
		internal float m_referenceAngle;
		internal Vec3 m_impulse;
		internal float m_motorImpulse;
		internal float m_lowerTranslation;
		internal float m_upperTranslation;
		internal float m_maxMotorForce;
		internal float m_motorSpeed;
		internal bool m_enableLimit;
		internal bool m_enableMotor;
		internal LimitState m_limitState;

		// Solver temp
		internal int m_indexA;
		internal int m_indexB;
		internal Vec2 m_localCenterA;
		internal Vec2 m_localCenterB;
		internal float m_invMassA;
		internal float m_invMassB;
		internal float m_invIA;
		internal float m_invIB;
		internal Vec2 m_axis, m_perp;
		internal float m_s1, m_s2;
		internal float m_a1, m_a2;
		internal Mat33 m_K;
		internal float m_motorMass;
	};
}
