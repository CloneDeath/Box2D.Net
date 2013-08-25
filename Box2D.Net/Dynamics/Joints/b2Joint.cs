using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// The base joint class. Joints are used to constraint two bodies together in
	/// various fashions. Some joints also feature limits and motors.
	public abstract class Joint
	{
		protected JointType m_type;
		protected Joint m_prev;
		protected Joint m_next;
		internal List<JointEdge> m_edgeA;
		internal List<JointEdge> m_edgeB;
		internal Body m_bodyA;
		internal Body m_bodyB;

		internal int m_index;

		internal bool m_islandFlag;
		internal bool m_collideConnected;

		protected object m_userData;

		protected Joint(JointDef def){
			Utilities.Assert(def.bodyA != def.bodyB);

			m_type = def.type;
			m_prev = null;
			m_next = null;
			m_bodyA = def.bodyA;
			m_bodyB = def.bodyB;
			m_index = 0;
			m_collideConnected = def.collideConnected;
			m_islandFlag = false;
			m_userData = def.userData;

			m_edgeA = new List<JointEdge>();
			m_edgeB = new List<JointEdge>();
		}
		~Joint() {}

		internal abstract void InitVelocityConstraints(SolverData data);
		internal abstract void SolveVelocityConstraints(SolverData data);

		// This returns true if the position errors are within tolerance.
		internal abstract bool SolvePositionConstraints(SolverData data);

		/// Get the type of the concrete joint.
		public JointType GetJointType(){
			return m_type;
		}

		/// Get the first body attached to this joint.
		public Body GetBodyA(){
			return m_bodyA;
		}

		/// Get the second body attached to this joint.
		public Body GetBodyB(){
			return m_bodyB;
		}

		/// Get the anchor point on bodyA in world coordinates.
		public abstract Vec2 GetAnchorA();

		/// Get the anchor point on bodyB in world coordinates.
		public abstract Vec2 GetAnchorB();

		/// Get the reaction force on bodyB at the joint anchor in Newtons.
		public abstract Vec2 GetReactionForce(float inv_dt);

		/// Get the reaction torque on bodyB in N*m.
		public abstract float GetReactionTorque(float inv_dt);

		/// Get the next joint the world joint list.
		public Joint GetNext(){
			return m_next;
		}

		/// Gets/sets the user data.
		public object UserData {
			get {
				return m_userData;
			}
			set {
				m_userData = value;
			}
		}

		/// Short-cut function to determine if either body is inactive.
		public bool IsActive(){
			return m_bodyA.IsActive() && m_bodyB.IsActive();
		}

		/// Get collide connected.
		/// Note: modifying the collide connect flag won't work correctly because
		/// the flag is only checked when fixture AABBs begin to overlap.
		public bool GetCollideConnected(){
			return m_collideConnected;
		}

		/// Dump this joint to the log file.
		public virtual void Dump() { Settings.Log("// Dump is not supported for this joint type.\n"); }

		/// Shift the origin for any points stored in world coordinates.
		public virtual void ShiftOrigin(Vec2 newOrigin) {
			
		}

		internal static Joint Create(JointDef def){
			Joint joint = null;

			switch (def.type)
			{
			case JointType.e_distanceJoint:
			    {
					joint = new DistanceJoint((DistanceJointDef)def);
			    }
			    break;

			case JointType.e_mouseJoint:
			    {
			        joint = new MouseJoint((MouseJointDef)def);
			    }
			    break;

			case JointType.e_prismaticJoint:
			    {
			        joint = new PrismaticJoint((PrismaticJointDef)def);
			    }
			    break;

			case JointType.e_revoluteJoint:
			    {
					joint = new RevoluteJoint((RevoluteJointDef)def);
			    }
			    break;

			case JointType.e_pulleyJoint:
			    {
					joint = new PulleyJoint((PulleyJointDef)def);
			    }
			    break;

			case JointType.e_gearJoint:
			    {
					joint = new GearJoint((GearJointDef)def);
			    }
			    break;

			case JointType.e_wheelJoint:
			    {
					joint = new WheelJoint((WheelJointDef)def);
			    }
			    break;

			case JointType.e_weldJoint:
			    {
					joint = new WeldJoint((WeldJointDef)def);
			    }
			    break;
        
			case JointType.e_frictionJoint:
			    {
			        joint = new FrictionJoint((FrictionJointDef)def);
			    }
			    break;

			case JointType.e_ropeJoint:
			    {
					throw new NotImplementedException();
					//joint = new RopeJoint((RopeJointDef)def);
			    }
			    break;

			case JointType.e_motorJoint:
			    {
			        joint = new MotorJoint((MotorJointDef)def);
			    }
			    break;

			default:
			    Utilities.Assert(false);
			    break;
			}

			return joint;
		}
		protected static void Destroy(Joint joint){
			throw new NotImplementedException();
			//joint.~Joint();
			//switch (joint.m_type)
			//{
			//case JointType.e_distanceJoint:
			//    allocator.Free(joint, sizeof(DistanceJoint));
			//    break;

			//case JointType.e_mouseJoint:
			//    allocator.Free(joint, sizeof(MouseJoint));
			//    break;

			//case JointType.e_prismaticJoint:
			//    allocator.Free(joint, sizeof(PrismaticJoint));
			//    break;

			//case JointType.e_revoluteJoint:
			//    allocator.Free(joint, sizeof(RevoluteJoint));
			//    break;

			//case JointType.e_pulleyJoint:
			//    allocator.Free(joint, sizeof(PulleyJoint));
			//    break;

			//case JointType.e_gearJoint:
			//    allocator.Free(joint, sizeof(GearJoint));
			//    break;

			//case JointType.e_wheelJoint:
			//    allocator.Free(joint, sizeof(WheelJoint));
			//    break;
    
			//case JointType.e_weldJoint:
			//    allocator.Free(joint, sizeof(WeldJoint));
			//    break;

			//case JointType.e_frictionJoint:
			//    allocator.Free(joint, sizeof(FrictionJoint));
			//    break;

			//case JointType.e_ropeJoint:
			//    allocator.Free(joint, sizeof(RopeJoint));
			//    break;

			//case JointType.e_motorJoint:
			//    allocator.Free(joint, sizeof(MotorJoint));
			//    break;

			//default:
			//    Utilities.Assert(false);
			//    break;
			//}
		}

		
	}
}
