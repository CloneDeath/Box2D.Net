using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A wheel joint. This joint provides two degrees of freedom: translation
	/// along an axis fixed in bodyA and rotation in the plane. You can use a
	/// joint limit to restrict the range of motion and a joint motor to drive
	/// the rotation or to model rotational friction.
	/// This joint is designed for vehicle suspensions.
	class b2WheelJoint : b2Joint
	{
		public override b2Vec2 GetAnchorA(){
			return m_bodyA.GetWorldPoint(m_localAnchorA);
		}
		public override b2Vec2 GetAnchorB(){
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		public override b2Vec2 GetReactionForce(float inv_dt){
			return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
		}
		public override float GetReactionTorque(float inv_dt){
			return inv_dt * m_motorImpulse;
		}

		/// The local anchor point relative to bodyA's origin.
		public b2Vec2 GetLocalAnchorA() { return m_localAnchorA; }

		/// The local anchor point relative to bodyB's origin.
		public b2Vec2 GetLocalAnchorB()  { return m_localAnchorB; }

		/// The local joint axis relative to bodyA.
		public b2Vec2 GetLocalAxisA() { return m_localXAxisA; }

		/// Get the current joint translation, usually in meters.
		public float GetJointTranslation(){
			b2Body bA = m_bodyA;
			b2Body bB = m_bodyB;

			b2Vec2 pA = bA.GetWorldPoint(m_localAnchorA);
			b2Vec2 pB = bB.GetWorldPoint(m_localAnchorB);
			b2Vec2 d = pB - pA;
			b2Vec2 axis = bA.GetWorldVector(m_localXAxisA);

			float translation = Utilities.b2Dot(d, axis);
			return translation;
		}

		/// Get the current joint translation speed, usually in meters per second.
		public float GetJointSpeed(){
			float wA = m_bodyA.m_angularVelocity;
			float wB = m_bodyB.m_angularVelocity;
			return wB - wA;
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

		/// Set the motor speed, usually in radians per second.
		public void SetMotorSpeed(float speed){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_motorSpeed = speed;
		}

		/// Get the motor speed, usually in radians per second.
		public float GetMotorSpeed(){
			return m_motorSpeed;
		}

		/// Set/Get the maximum motor force, usually in N-m.
		public void SetMaxMotorTorque(float torque){
			m_bodyA.SetAwake(true);
			m_bodyB.SetAwake(true);
			m_maxMotorTorque = torque;
		}
		public float GetMaxMotorTorque(){
			return m_maxMotorTorque; 
		}

		/// Get the current motor torque given the inverse time step, usually in N-m.
		public float GetMotorTorque(float inv_dt){
			return inv_dt * m_motorImpulse;
		}

		/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
		public void SetSpringFrequencyHz(float hz){
			m_frequencyHz = hz;
		}
		public float GetSpringFrequencyHz(){
			return m_frequencyHz;
		}


		/// Set/Get the spring damping ratio
		public void SetSpringDampingRatio(float ratio){
			m_dampingRatio = ratio;
		}
		public float GetSpringDampingRatio(){
			return m_dampingRatio;
		}

		/// Dump to b2Settings.b2Log
		public void Dump(){
			int indexA = m_bodyA.m_islandIndex;
			int indexB = m_bodyB.m_islandIndex;

			b2Settings.b2Log("  b2WheelJointDef jd;\n");
			b2Settings.b2Log("  jd.bodyA = bodies[%d];\n", indexA);
			b2Settings.b2Log("  jd.bodyB = bodies[%d];\n", indexB);
			b2Settings.b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
			b2Settings.b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
			b2Settings.b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
			b2Settings.b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
			b2Settings.b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
			b2Settings.b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
			b2Settings.b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
			b2Settings.b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
			b2Settings.b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
			b2Settings.b2Log("  joints[%d] = m_world.CreateJoint(&jd);\n", m_index);
		}

		internal b2WheelJoint(b2WheelJointDef def): base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_localXAxisA = def.localAxisA;
			m_localYAxisA = Utilities.b2Cross(1.0f, m_localXAxisA);

			m_mass = 0.0f;
			m_impulse = 0.0f;
			m_motorMass = 0.0f;
			m_motorImpulse = 0.0f;
			m_springMass = 0.0f;
			m_springImpulse = 0.0f;

			m_maxMotorTorque = def.maxMotorTorque;
			m_motorSpeed = def.motorSpeed;
			m_enableMotor = def.enableMotor;

			m_frequencyHz = def.frequencyHz;
			m_dampingRatio = def.dampingRatio;

			m_bias = 0.0f;
			m_gamma = 0.0f;

			m_ax.SetZero();
			m_ay.SetZero();
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

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			b2Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			b2Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			b2Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			b2Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			b2Rot qA = new b2Rot(aA); b2Rot qB = new b2Rot(aB);

			// Compute the effective masses.
			b2Vec2 rA = Utilities.b2Mul(qA, m_localAnchorA - m_localCenterA);
			b2Vec2 rB = Utilities.b2Mul(qB, m_localAnchorB - m_localCenterB);
			b2Vec2 d = cB + rB - cA - rA;

			// Point to line constraint
			{
				m_ay = Utilities.b2Mul(qA, m_localYAxisA);
				m_sAy = Utilities.b2Cross(d + rA, m_ay);
				m_sBy = Utilities.b2Cross(rB, m_ay);

				m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

				if (m_mass > 0.0f)
				{
					m_mass = 1.0f / m_mass;
				}
			}

			// Spring constraint
			m_springMass = 0.0f;
			m_bias = 0.0f;
			m_gamma = 0.0f;
			if (m_frequencyHz > 0.0f)
			{
				m_ax = Utilities.b2Mul(qA, m_localXAxisA);
				m_sAx = Utilities.b2Cross(d + rA, m_ax);
				m_sBx = Utilities.b2Cross(rB, m_ax);

				float invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

				if (invMass > 0.0f)
				{
					m_springMass = 1.0f / invMass;

					float C = Utilities.b2Dot(d, m_ax);

					// Frequency
					float omega = 2.0f * (float)Math.PI * m_frequencyHz;

					// Damping coefficient
					float df = 2.0f * m_springMass * m_dampingRatio * omega;

					// Spring stiffness
					float k = m_springMass * omega * omega;

					// magic formulas
					float h = data.step.dt;
					m_gamma = h * (df + h * k);
					if (m_gamma > 0.0f)
					{
						m_gamma = 1.0f / m_gamma;
					}

					m_bias = C * h * k * m_gamma;

					m_springMass = invMass + m_gamma;
					if (m_springMass > 0.0f)
					{
						m_springMass = 1.0f / m_springMass;
					}
				}
			}
			else
			{
				m_springImpulse = 0.0f;
			}

			// Rotational motor
			if (m_enableMotor)
			{
				m_motorMass = iA + iB;
				if (m_motorMass > 0.0f)
				{
					m_motorMass = 1.0f / m_motorMass;
				}
			}
			else
			{
				m_motorMass = 0.0f;
				m_motorImpulse = 0.0f;
			}

			if (data.step.warmStarting)
			{
				// Account for variable time step.
				m_impulse *= data.step.dtRatio;
				m_springImpulse *= data.step.dtRatio;
				m_motorImpulse *= data.step.dtRatio;

				b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
				float LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
				float LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

				vA -= m_invMassA * P;
				wA -= m_invIA * LA;

				vB += m_invMassB * P;
				wB += m_invIB * LB;
			}
			else
			{
				m_impulse = 0.0f;
				m_springImpulse = 0.0f;
				m_motorImpulse = 0.0f;
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}
		internal override void SolveVelocityConstraints(b2SolverData data){
			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			b2Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			b2Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			// Solve spring constraint
			{
				float Cdot = Utilities.b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
				float impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
				m_springImpulse += impulse;

				b2Vec2 P = impulse * m_ax;
				float LA = impulse * m_sAx;
				float LB = impulse * m_sBx;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
			}

			// Solve rotational motor constraint
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

			// Solve point to line constraint
			{
				float Cdot = Utilities.b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
				float impulse = -m_mass * Cdot;
				m_impulse += impulse;

				b2Vec2 P = impulse * m_ay;
				float LA = impulse * m_sAy;
				float LB = impulse * m_sBy;

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
		internal override bool SolvePositionConstraints(b2SolverData data){
			b2Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			b2Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			b2Rot qA = new b2Rot(aA);
			b2Rot qB = new b2Rot(aB);

			b2Vec2 rA = Utilities.b2Mul(qA, m_localAnchorA - m_localCenterA);
			b2Vec2 rB = Utilities.b2Mul(qB, m_localAnchorB - m_localCenterB);
			b2Vec2 d = (cB - cA) + rB - rA;

			b2Vec2 ay = Utilities.b2Mul(qA, m_localYAxisA);

			float sAy = Utilities.b2Cross(d + rA, ay);
			float sBy = Utilities.b2Cross(rB, ay);

			float C = Utilities.b2Dot(d, ay);

			float k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

			float impulse;
			if (k != 0.0f)
			{
				impulse = - C / k;
			}
			else
			{
				impulse = 0.0f;
			}

			b2Vec2 P = impulse * ay;
			float LA = impulse * sAy;
			float LB = impulse * sBy;

			cA -= m_invMassA * P;
			aA -= m_invIA * LA;
			cB += m_invMassB * P;
			aB += m_invIB * LB;

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return Math.Abs(C) <=b2Settings.b2_linearSlop;
		}

		float m_frequencyHz;
		float m_dampingRatio;

		// Solver shared
		b2Vec2 m_localAnchorA;
		b2Vec2 m_localAnchorB;
		b2Vec2 m_localXAxisA;
		b2Vec2 m_localYAxisA;

		float m_impulse;
		float m_motorImpulse;
		float m_springImpulse;

		float m_maxMotorTorque;
		float m_motorSpeed;
		bool m_enableMotor;

		// Solver temp
		int m_indexA;
		int m_indexB;
		b2Vec2 m_localCenterA;
		b2Vec2 m_localCenterB;
		float m_invMassA;
		float m_invMassB;
		float m_invIA;
		float m_invIB;

		b2Vec2 m_ax, m_ay;
		float m_sAx, m_sBx;
		float m_sAy, m_sBy;

		float m_mass;
		float m_motorMass;
		float m_springMass;

		float m_bias;
		float m_gamma;
	};
}
