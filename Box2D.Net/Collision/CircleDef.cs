using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D
{
	/// <summary>
	/// This structure is used to build a fixture with a circle shape.
	/// </summary>
	public class CircleDef : FixtureDef
	{
		public Vec2 LocalPosition
		{
			get
			{
				return circle.m_position;
			}
			set
			{
				circle.m_position = value;
			}
		}
		public float Radius
		{
			get
			{
				return circle.m_radius;
			}
			set
			{
				circle.m_radius = value;
			}
		}

		public CircleDef()
		{
			Type = ShapeType.Circle;
			LocalPosition = new Vec2();
			shape = new CircleShape();
			Radius = 1.0f;
		}

		private CircleShape circle
		{
			get
			{
				return (CircleShape)shape;
			}
		}
	}
}
