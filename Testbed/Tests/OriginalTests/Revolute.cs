using System;
using System.Collections.Generic;
using System.Linq;
using System.Text; 
using Testbed.Framework;
using Box2D;
using GLImp;
using OpenTK.Input;

namespace Testbed.Tests {
	class Revolute : Test
	{
		public Revolute()
		{
			Body ground = null;
			{
				BodyDef bd = new BodyDef();
				ground = m_world.CreateBody(bd);

				EdgeShape shape = new EdgeShape();
				shape.Set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));

				FixtureDef fd = new FixtureDef();
				fd.shape = shape;
				//fd.Filter.CategoryBits = 2;

				ground.CreateFixture(fd);
			}

			{
				CircleShape shape = new CircleShape();
				shape.m_radius = 0.5f;
				shape.Density = 5;

				BodyDef bd = new BodyDef();
				bd.type = BodyType._dynamicBody;

				RevoluteJointDef rjd = new RevoluteJointDef();

				bd.Position.Set(-10.0f, 20.0f);
				Body body = m_world.CreateBody(bd);
				body.CreateFixture(shape);

				float w = 100.0f;
				body.SetAngularVelocity(w);
				body.SetLinearVelocity(new Vec2(-8.0f * w, 0.0f));

				rjd.Initialize(ground, body, new Vec2(-10.0f, 12.0f));
				rjd.motorSpeed = 1.0f * (float)Math.PI;
				rjd.maxMotorTorque = 10000.0f;
				rjd.enableMotor = false;
				rjd.lowerAngle = -0.25f * (float)Math.PI;
				rjd.upperAngle = 0.5f * (float)Math.PI;
				rjd.enableLimit = true;
				rjd.collideConnected = true;

				m_joint = (RevoluteJoint)m_world.CreateJoint(rjd);
			}

			{
				CircleShape circle_shape = new CircleShape();
				circle_shape.m_radius = 3.0f;

				BodyDef circle_bd = new BodyDef();
				circle_bd.type = BodyType._dynamicBody;
				circle_bd.Position.Set(5.0f, 30.0f);

				FixtureDef fd = new FixtureDef();
				fd.Density = 5.0f;
				fd.Filter.MaskBits = 1;
				fd.shape = circle_shape;

				m_ball = m_world.CreateBody(circle_bd);
				m_ball.CreateFixture(fd);

				PolygonShape polygon_shape = new PolygonShape();
				polygon_shape.SetAsBox(10.0f, 0.2f, new Vec2(-10.0f, 0.0f), 0.0f);
				polygon_shape.Density = 2;

				BodyDef polygon_bd = new BodyDef();
				polygon_bd.Position.Set(20.0f, 10.0f);
				polygon_bd.type = BodyType._dynamicBody;
				polygon_bd.bullet = true;
				Body polygon_body = m_world.CreateBody(polygon_bd);
				
				polygon_body.CreateFixture(polygon_shape);

				RevoluteJointDef rjd = new RevoluteJointDef();
				rjd.Initialize(ground, polygon_body, new Vec2(20.0f, 10.0f));
				rjd.lowerAngle = -0.25f * (float)Math.PI;
				rjd.upperAngle = 0.0f * (float)Math.PI;
				rjd.enableLimit = true;
				m_world.CreateJoint(rjd);
			}

			// Tests mass computation of a small object far from the origin
			{
				BodyDef bodyDef = new BodyDef();
				bodyDef.type = BodyType._dynamicBody;
				Body body = m_world.CreateBody(bodyDef);
		
				PolygonShape polyShape = new PolygonShape();
				Vec2[] verts = new Vec2[3];
				verts[0].Set( 17.63f, 36.31f );
				verts[1].Set( 17.52f, 36.69f );
				verts[2].Set( 17.19f, 36.36f );
				polyShape.Set(verts, 3);
		
				FixtureDef polyFixtureDef = new FixtureDef();
				polyFixtureDef.shape = polyShape;
				polyFixtureDef.Density = 1;

				body.CreateFixture(polyFixtureDef);	//assertion hits inside here
			}

		}

		public override void Keyboard()
		{
			if (KeyboardManager.IsPressed(Key.L)){
				m_joint.EnableLimit(!m_joint.IsLimitEnabled());
			}

			if (KeyboardManager.IsPressed(Key.M)) {
				m_joint.EnableMotor(!m_joint.IsMotorEnabled());
			}
		}

		public override void Step(TestSettings settings)
		{
			base.Step(settings);
			m_debugDraw.DrawString("Keys: (l) limits, (m) motor");
			

			//if (m_stepCount == 360)
			//{
			//	m_ball.SetTransform(new Vec2(0.0f, 0.5f), 0.0f);
			//}

			//float torque1 = m_joint1.GetMotorTorque();
			//m_debugDraw.DrawString("Motor Torque = %4.0f, %4.0f : Motor Force = %4.0f", (float) torque1, (float) torque2, (float) force3);
			//
		}

		public static Test Create()
		{
			return new Revolute();
		}

		Body m_ball;
		RevoluteJoint m_joint;
	};
}
