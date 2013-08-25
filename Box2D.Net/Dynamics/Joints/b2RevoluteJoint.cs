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
	class b2RevoluteJoint : b2Joint
	{
	
		public b2Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public b2Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		/// The local anchor point relative to bodyA's origin.
		public b2Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public b2Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// Get the reference angle.
		public float GetReferenceAngle() { return m_referenceAngle; }

		/// Get the current joint angle in radians.
		public float GetJointAngle(){
			b2Body* bA = m_bodyA;
			b2Body* bB = m_bodyB;
			return bB.m_sweep.a - bA.m_sweep.a - m_referenceAngle;
		}

		/// Get the current joint angle speed in radians per second.
		public float GetJointSpeed(){
			b2Body* bA = m_bodyA;
			b2Body* bB = m_bodyB;
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
				m_impulse.z = 0.0f;
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
				m_impulse.z = 0.0f;
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
		public b2Vec2 GetReactionForce(float inv_dt){
			b2Vec2 P(m_impulse.x, m_impulse.y);
			return inv_dt * P;
		}

		/// Get the reaction torque due to the joint limit given the inverse time step.
		/// Unit is N*m.
		public float GetReactionTorque(float inv_dt){
			return inv_dt * m_impulse.z;
		}

		/// Get the current motor torque given the inverse time step.
		/// Unit is N*m.
		public float GetMotorTorque(float inv_dt){
			return inv_dt * m_motorImpulse;
		}

		/// Dump to b2Settings.b2Log.
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			b2Settings.b2Log("  b2RevoluteJointDef jd;\n");
			b2Settings.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
			b2Settings.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
			b2Settings.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
			b2Settings.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
			b2Settings.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
			b2Settings.b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
			b2Settings.b2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
			b2Settings.b2Log("  jd.lowerAngle = %.15lef;\n", m_lowerAngle);
			b2Settings.b2Log("  jd.upperAngle = %.15lef;\n", m_upperAngle);
			b2Settings.b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
			b2Settings.b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
			b2Settings.b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
			b2Settings.b2Log("  joints[%d] = m_world.CreateJoint(&jd);\n", m_index);
		}

		internal b2RevoluteJoint(b2RevoluteJointDef def): base(def)
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
			m_limitState = e_inactiveLimit;
		}

		void InitVelocityConstraints(b2SolverData data){
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

			bool fixedRotation = (iA + iB == 0.0f);

			m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
			m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
			m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB;
			m_mass.ex.y = m_mass.ey.x;
			m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
			m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
			m_mass.ex.z = m_mass.ez.x;
			m_mass.ey.z = m_mass.ez.y;
			m_mass.ez.z = iA + iB;

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
				if (Math.Abs(m_upperAngle - m_lowerAngle) < 2.0f * b2Settings.b2_angularSlop)
				{
					m_limitState = e_equalLimits;
				}
				else if (jointAngle <= m_lowerAngle)
				{
					if (m_limitState != e_atLowerLimit)
					{
						m_impulse.z = 0.0f;
					}
					m_limitState = e_atLowerLimit;
				}
				else if (jointAngle >= m_upperAngle)
				{
					if (m_limitState != e_atUpperLimit)
					{
						m_impulse.z = 0.0f;
					}
					m_limitState = e_atUpperLimit;
				}
				else
				{
					m_limitState = e_inactiveLimit;
					m_impulse.z = 0.0f;
				}
			}
			else
			{
				m_limitState = e_inactiveLimit;
			}

			if (data.step.warmStarting)
			{
				// Scale impulses to support a variable time step.
				m_impulse *= data.step.dtRatio;
				m_motorImpulse *= data.step.dtRatio;

				b2Vec2 P(m_impulse.x, m_impulse.y);

				vA -= mA * P;
				wA -= iA * (Utilities.b2Cross(m_rA, P) + m_motorImpulse + m_impulse.z);

				vB += mB * P;
				wB += iB * (Utilities.b2Cross(m_rB, P) + m_motorImpulse + m_impulse.z);
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
		void SolveVelocityConstraints(b2SolverData data){
			b2Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			b2Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			bool fixedRotation = (iA + iB == 0.0f);

			// Solve motor constraint.
			if (m_enableMotor && m_limitState != e_equalLimits && fixedRotation == false)
			{
				float Cdot = wB - wA - m_motorSpeed;
				float impulse = -m_motorMass * Cdot;
				float oldImpulse = m_motorImpulse;
				float maxImpulse = data.step.dt * m_maxMotorTorque;
				m_motorImpulse = Utilities.b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = m_motorImpulse - oldImpulse;

				wA -= iA * impulse;
				wB += iB * impulse;
			}

			// Solve limit constraint.
			if (m_enableLimit && m_limitState != e_inactiveLimit && fixedRotation == false)
			{
				b2Vec2 Cdot1 = vB + Utilities.b2Cross(wB, m_rB) - vA - Utilities.b2Cross(wA, m_rA);
				float Cdot2 = wB - wA;
				b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

				b2Vec3 impulse = -m_mass.Solve33(Cdot);

				if (m_limitState == e_equalLimits)
				{
					m_impulse += impulse;
				}
				else if (m_limitState == e_atLowerLimit)
				{
					float newImpulse = m_impulse.z + impulse.z;
					if (newImpulse < 0.0f)
					{
						b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
						b2Vec2 reduced = m_mass.Solve22(rhs);
						impulse.x = reduced.x;
						impulse.y = reduced.y;
						impulse.z = -m_impulse.z;
						m_impulse.x += reduced.x;
						m_impulse.y += reduced.y;
						m_impulse.z = 0.0f;
					}
					else
					{
						m_impulse += impulse;
					}
				}
				else if (m_limitState == e_atUpperLimit)
				{
					float newImpulse = m_impulse.z + impulse.z;
					if (newImpulse > 0.0f)
					{
						b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
						b2Vec2 reduced = m_mass.Solve22(rhs);
						impulse.x = reduced.x;
						impulse.y = reduced.y;
						impulse.z = -m_impulse.z;
						m_impulse.x += reduced.x;
						m_impulse.y += reduced.y;
						m_impulse.z = 0.0f;
					}
					else
					{
						m_impulse += impulse;
					}
				}

				b2Vec2 P(impulse.x, impulse.y);

				vA -= mA * P;
				wA -= iA * (Utilities.b2Cross(m_rA, P) + impulse.z);

				vB += mB * P;
				wB += iB * (Utilities.b2Cross(m_rB, P) + impulse.z);
			}
			else
			{
				// Solve point-to-point constraint
				b2Vec2 Cdot = vB + Utilities.b2Cross(wB, m_rB) - vA - Utilities.b2Cross(wA, m_rA);
				b2Vec2 impulse = m_mass.Solve22(-Cdot);

				m_impulse.x += impulse.x;
				m_impulse.y += impulse.y;

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
		bool SolvePositionConstraints(b2SolverData data){
			b2Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			b2Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			b2Rot qA = new b2Rot(aA);
			b2Rot qB = new b2Rot(aB);

			float angularError = 0.0f;
			float positionError = 0.0f;

			bool fixedRotation = (m_invIA + m_invIB == 0.0f);

			// Solve angular limit constraint.
			if (m_enableLimit && m_limitState != e_inactiveLimit && fixedRotation == false)
			{
				float angle = aB - aA - m_referenceAngle;
				float limitImpulse = 0.0f;

				if (m_limitState == e_equalLimits)
				{
					// Prevent large angular corrections
					float C = Utilities.b2Clamp(angle - m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
					limitImpulse = -m_motorMass * C;
					angularError = Math.Abs(C);
				}
				else if (m_limitState == e_atLowerLimit)
				{
					float C = angle - m_lowerAngle;
					angularError = -C;

					// Prevent large angular corrections and allow some slop.
					C = Utilities.b2Clamp(C + b2Settings.b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
					limitImpulse = -m_motorMass * C;
				}
				else if (m_limitState == e_atUpperLimit)
				{
					float C = angle - m_upperAngle;
					angularError = C;

					// Prevent large angular corrections and allow some slop.
					C = Utilities.b2Clamp(C - b2Settings.b2_angularSlop, 0.0f, b2_maxAngularCorrection);
					limitImpulse = -m_motorMass * C;
				}

				aA -= m_invIA * limitImpulse;
				aB += m_invIB * limitImpulse;
			}

			// Solve point-to-point constraint.
			{
				qA.Set(aA);
				qB.Set(aB);
				b2Vec2 rA = Utilities.b2Mul(qA, m_localAnchorA - m_localCenterA);
				b2Vec2 rB = Utilities.b2Mul(qB, m_localAnchorB - m_localCenterB);

				b2Vec2 C = cB + rB - cA - rA;
				positionError = C.Length();

				float mA = m_invMassA, mB = m_invMassB;
				float iA = m_invIA, iB = m_invIB;

				b2Mat22 K;
				K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
				K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
				K.ey.x = K.ex.y;
				K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

				b2Vec2 impulse = -K.Solve(C);

				cA -= mA * impulse;
				aA -= iA * Utilities.b2Cross(rA, impulse);

				cB += mB * impulse;
				aB += iB * Utilities.b2Cross(rB, impulse);
			}

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;
	
			return positionError <=b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
		}

		// Solver shared
		b2Vec2 m_localAnchorA;
		b2Vec2 m_localAnchorB;
		b2Vec3 m_impulse;
		float m_motorImpulse;

		bool m_enableMotor;
		float m_maxMotorTorque;
		float m_motorSpeed;

		bool m_enableLimit;
		float m_referenceAngle;
		float m_lowerAngle;
		float m_upperAngle;

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
		b2Mat33 m_mass;			// effective mass for point-to-point constraint.
		float m_motorMass;	// effective mass for motor/limit angular constraint.
		b2LimitState m_limitState;
	};
}
