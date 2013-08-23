﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// The base joint class. Joints are used to constraint two bodies together in
	/// various fashions. Some joints also feature limits and motors.
	public abstract class b2Joint
	{
		protected b2JointType m_type;
		protected b2Joint m_prev;
		protected b2Joint m_next;
		protected b2JointEdge m_edgeA;
		protected b2JointEdge m_edgeB;
		protected b2Body m_bodyA;
		protected b2Body m_bodyB;

		protected int m_index;

		protected bool m_islandFlag;
		protected bool m_collideConnected;

		protected object m_userData;

		protected b2Joint(b2JointDef def){
			throw new NotImplementedException();
			//Utilities.Assert(def.bodyA != def.bodyB);

			//m_type = def.type;
			//m_prev = null;
			//m_next = null;
			//m_bodyA = def.bodyA;
			//m_bodyB = def.bodyB;
			//m_index = 0;
			//m_collideConnected = def.collideConnected;
			//m_islandFlag = false;
			//m_userData = def.userData;

			//m_edgeA.joint = null;
			//m_edgeA.other = null;
			//m_edgeA.prev = null;
			//m_edgeA.next = null;

			//m_edgeB.joint = null;
			//m_edgeB.other = null;
			//m_edgeB.prev = null;
			//m_edgeB.next = null;
		}
		~b2Joint() {}

		protected abstract void InitVelocityConstraints(b2SolverData data);
		protected abstract void SolveVelocityConstraints(b2SolverData data);

		// This returns true if the position errors are within tolerance.
		protected abstract bool SolvePositionConstraints(b2SolverData data);

		/// Get the type of the concrete joint.
		public b2JointType GetType(){
			return m_type;
		}

		/// Get the first body attached to this joint.
		public b2Body GetBodyA(){
			return m_bodyA;
		}

		/// Get the second body attached to this joint.
		public b2Body GetBodyB(){
			return m_bodyB;
		}

		/// Get the anchor point on bodyA in world coordinates.
		public abstract b2Vec2 GetAnchorA();

		/// Get the anchor point on bodyB in world coordinates.
		public abstract b2Vec2 GetAnchorB();

		/// Get the reaction force on bodyB at the joint anchor in Newtons.
		public abstract b2Vec2 GetReactionForce(float inv_dt);

		/// Get the reaction torque on bodyB in N*m.
		public abstract float GetReactionTorque(float inv_dt);

		/// Get the next joint the world joint list.
		public b2Joint GetNext(){
			return m_next;
		}

		/// Get the user data pointer.
		public object GetUserData(){
			return m_userData;
		}


		/// Set the user data pointer.
		public void SetUserData(object data){
			m_userData = data;
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
		public virtual void Dump() { b2Settings.b2Log("// Dump is not supported for this joint type.\n"); }

		/// Shift the origin for any points stored in world coordinates.
		public virtual void ShiftOrigin(b2Vec2 newOrigin) {
			throw new NotImplementedException();
			//B2_NOT_USED(newOrigin);  
		}

		protected static b2Joint Create(b2JointDef def){
			throw new NotImplementedException();
			//b2Joint* joint = null;

			//switch (def.type)
			//{
			//case e_distanceJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2DistanceJoint));
			//        joint = new (mem) b2DistanceJoint((b2DistanceJointDef*)def);
			//    }
			//    break;

			//case e_mouseJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2MouseJoint));
			//        joint = new (mem) b2MouseJoint((b2MouseJointDef*)def);
			//    }
			//    break;

			//case e_prismaticJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2PrismaticJoint));
			//        joint = new (mem) b2PrismaticJoint((b2PrismaticJointDef*)def);
			//    }
			//    break;

			//case e_revoluteJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2RevoluteJoint));
			//        joint = new (mem) b2RevoluteJoint((b2RevoluteJointDef*)def);
			//    }
			//    break;

			//case e_pulleyJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2PulleyJoint));
			//        joint = new (mem) b2PulleyJoint((b2PulleyJointDef*)def);
			//    }
			//    break;

			//case e_gearJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2GearJoint));
			//        joint = new (mem) b2GearJoint((b2GearJointDef*)def);
			//    }
			//    break;

			//case e_wheelJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2WheelJoint));
			//        joint = new (mem) b2WheelJoint((b2WheelJointDef*)def);
			//    }
			//    break;

			//case e_weldJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2WeldJoint));
			//        joint = new (mem) b2WeldJoint((b2WeldJointDef*)def);
			//    }
			//    break;
        
			//case e_frictionJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2FrictionJoint));
			//        joint = new (mem) b2FrictionJoint((b2FrictionJointDef*)def);
			//    }
			//    break;

			//case e_ropeJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2RopeJoint));
			//        joint = new (mem) b2RopeJoint((b2RopeJointDef*)def);
			//    }
			//    break;

			//case e_motorJoint:
			//    {
			//        void* mem = allocator.Allocate(sizeof(b2MotorJoint));
			//        joint = new (mem) b2MotorJoint((b2MotorJointDef*)def);
			//    }
			//    break;

			//default:
			//    Utilities.Assert(false);
			//    break;
			//}

			//return joint;
		}
		protected static void Destroy(b2Joint joint){
			throw new NotImplementedException();
			//joint.~b2Joint();
			//switch (joint.m_type)
			//{
			//case e_distanceJoint:
			//    allocator.Free(joint, sizeof(b2DistanceJoint));
			//    break;

			//case e_mouseJoint:
			//    allocator.Free(joint, sizeof(b2MouseJoint));
			//    break;

			//case e_prismaticJoint:
			//    allocator.Free(joint, sizeof(b2PrismaticJoint));
			//    break;

			//case e_revoluteJoint:
			//    allocator.Free(joint, sizeof(b2RevoluteJoint));
			//    break;

			//case e_pulleyJoint:
			//    allocator.Free(joint, sizeof(b2PulleyJoint));
			//    break;

			//case e_gearJoint:
			//    allocator.Free(joint, sizeof(b2GearJoint));
			//    break;

			//case e_wheelJoint:
			//    allocator.Free(joint, sizeof(b2WheelJoint));
			//    break;
    
			//case e_weldJoint:
			//    allocator.Free(joint, sizeof(b2WeldJoint));
			//    break;

			//case e_frictionJoint:
			//    allocator.Free(joint, sizeof(b2FrictionJoint));
			//    break;

			//case e_ropeJoint:
			//    allocator.Free(joint, sizeof(b2RopeJoint));
			//    break;

			//case e_motorJoint:
			//    allocator.Free(joint, sizeof(b2MotorJoint));
			//    break;

			//default:
			//    Utilities.Assert(false);
			//    break;
			//}
		}

		
	}
}
