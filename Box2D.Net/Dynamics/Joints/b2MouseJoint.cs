using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A mouse joint is used to make a point on a body track a
	/// specified world point. This a soft constraint with a maximum
	/// force. This allows the constraint to stretch and without
	/// applying huge forces.
	/// NOTE: this joint is not documented in the manual because it was
	/// developed to be used in the testbed. If you want to learn how to
	/// use the mouse joint, look at the testbed.
	public class MouseJoint : Joint
	{
		protected Vec2 m_localAnchorB;
		protected Vec2 m_targetA;
		protected float m_frequencyHz;
		protected float m_dampingRatio;
		protected float m_beta;
	
		// Solver shared
		protected Vec2 m_impulse;
		protected float m_maxForce;
		protected float m_gamma;

		// Solver temp
		protected int m_indexA;
		protected int m_indexB;
		protected Vec2 m_rB;
		protected Vec2 m_localCenterB;
		protected float m_invMassB;
		protected float m_invIB;
		protected Mat22 m_mass;
		protected Vec2 m_C;

		/// Implements Joint.
		public override Vec2 GetAnchorA(){
			return m_targetA;
		}

		/// Implements Joint.
		public override Vec2 GetAnchorB() {
			return m_bodyB.GetWorldPoint(m_localAnchorB);
		}

		/// Implements Joint.
		public override Vec2 GetReactionForce(float inv_dt) {
			return inv_dt * m_impulse;
		}

		/// Implements Joint.
		public override float GetReactionTorque(float inv_dt) {
			return inv_dt * 0.0f;
		}

		/// Use this to update the target point.
		public void SetTarget(Vec2 target){
			if (m_bodyB.IsAwake() == false)
			{
				m_bodyB.SetAwake(true);
			}
			m_targetA = target;
		}
		public Vec2 GetTarget(){
			return m_targetA;
		}

		/// Set/get the maximum force in Newtons.
		public void SetMaxForce(float force){
			m_maxForce = force;
		}
		public float GetMaxForce(){
			return m_maxForce;
		}

		/// Set/get the frequency in Hertz.
		public void SetFrequency(float hz){
			m_frequencyHz = hz;
		}
		public float GetFrequency(){
			return m_frequencyHz;
		}

		/// Set/get the damping ratio (dimensionless).
		public void SetDampingRatio(float ratio){
			m_dampingRatio = ratio;
		}
		public float GetDampingRatio(){
			return m_dampingRatio;
		}

		/// The mouse joint does not support dumping.
		public override void Dump() { Settings.Log("Mouse joint dumping is not supported.\n"); }

		/// Implement Joint::ShiftOrigin
		public override void ShiftOrigin(Vec2 newOrigin) {
			m_targetA -= newOrigin;
		}

		// p = attached point, m = mouse point
		// C = p - m
		// Cdot = v
		//      = v + cross(w, r)
		// J = [I r_skew]
		// Identity used:
		// w k % (rx i + ry j) = w * (-ry i + rx j)
		internal MouseJoint(MouseJointDef def) : base(def){
			Utilities.Assert(def.target.IsValid());
			Utilities.Assert(Utilities.IsValid(def.maxForce) && def.maxForce >= 0.0f);
			Utilities.Assert(Utilities.IsValid(def.frequencyHz) && def.frequencyHz >= 0.0f);
			Utilities.Assert(Utilities.IsValid(def.dampingRatio) && def.dampingRatio >= 0.0f);

			m_targetA = def.target;
			m_localAnchorB = Utilities.MulT(m_bodyB.GetTransform(), m_targetA);

			m_maxForce = def.maxForce;
			m_impulse.SetZero();

			m_frequencyHz = def.frequencyHz;
			m_dampingRatio = def.dampingRatio;

			m_beta = 0.0f;
			m_gamma = 0.0f;
		}

		internal override void InitVelocityConstraints(SolverData data) {
			throw new NotImplementedException();
			//m_indexB = m_bodyB.m_islandIndex;
			//m_localCenterB = m_bodyB.m_sweep.localCenter;
			//m_invMassB = m_bodyB.m_invMass;
			//m_invIB = m_bodyB.m_invI;

			//Vec2 cB = data.positions[m_indexB].c;
			//float aB = data.positions[m_indexB].a;
			//Vec2 vB = data.velocities[m_indexB].v;
			//float wB = data.velocities[m_indexB].w;

			//Rot qB(aB);

			//float mass = m_bodyB.GetMass();

			//// Frequency
			//float omega = 2.0f * (float)Math.PI * m_frequencyHz;

			//// Damping coefficient
			//float d = 2.0f * mass * m_dampingRatio * omega;

			//// Spring stiffness
			//float k = mass * (omega * omega);

			//// magic formulas
			//// gamma has units of inverse mass.
			//// beta has units of inverse time.
			//float h = data.step.dt;
			//Utilities.Assert(d + h * k > Single.Epsilon);
			//m_gamma = h * (d + h * k);
			//if (m_gamma != 0.0f)
			//{
			//    m_gamma = 1.0f / m_gamma;
			//}
			//m_beta = h * k * m_gamma;

			//// Compute the effective mass matrix.
			//m_rB = Utilities.Mul(qB, m_localAnchorB - m_localCenterB);

			//// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			////      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			////        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			//Mat22 K;
			//K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
			//K.ex.y = -m_invIB * m_rB.x * m_rB.y;
			//K.ey.x = K.ex.y;
			//K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;

			//m_mass = K.GetInverse();

			//m_C = cB + m_rB - m_targetA;
			//m_C *= m_beta;

			//// Cheat with some damping
			//wB *= 0.98f;

			//if (data.step.warmStarting)
			//{
			//    m_impulse *= data.step.dtRatio;
			//    vB += m_invMassB * m_impulse;
			//    wB += m_invIB * Utilities.Cross(m_rB, m_impulse);
			//}
			//else
			//{
			//    m_impulse.SetZero();
			//}

			//data.velocities[m_indexB].v = vB;
			//data.velocities[m_indexB].w = wB;
		}

		internal override void SolveVelocityConstraints(SolverData data) {
			throw new NotImplementedException();
			//Vec2 vB = data.velocities[m_indexB].v;
			//float wB = data.velocities[m_indexB].w;

			//// Cdot = v + cross(w, r)
			//Vec2 Cdot = vB + Utilities.Cross(wB, m_rB);
			//Vec2 impulse = Utilities.Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

			//Vec2 oldImpulse = m_impulse;
			//m_impulse += impulse;
			//float maxImpulse = data.step.dt * m_maxForce;
			//if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
			//{
			//    m_impulse *= maxImpulse / m_impulse.Length();
			//}
			//impulse = m_impulse - oldImpulse;

			//vB += m_invMassB * impulse;
			//wB += m_invIB * Utilities.Cross(m_rB, impulse);

			//data.velocities[m_indexB].v = vB;
			//data.velocities[m_indexB].w = wB;
		}

		internal override bool SolvePositionConstraints(SolverData data) {
			return true;
		}

		
	}
}
