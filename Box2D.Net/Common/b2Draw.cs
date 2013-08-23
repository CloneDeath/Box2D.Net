using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace Box2D {
	/// Implement and register this class with a b2World to provide debug drawing of physics
	/// entities in your game.
	public abstract class b2Draw
	{
		protected DrawFlags m_drawFlags;

		public b2Draw(){
			m_drawFlags = 0;
		}

		public virtual ~b2Draw() {}

		[Flags]
		public enum DrawFlags
		{
			e_shapeBit				= 0x0001,	///< draw shapes
			e_jointBit				= 0x0002,	///< draw joint connections
			e_aabbBit				= 0x0004,	///< draw axis aligned bounding boxes
			e_pairBit				= 0x0008,	///< draw broad-phase pairs
			e_centerOfMassBit		= 0x0010	///< draw center of mass frame
		};

		/// Set the drawing flags.
		public void SetFlags(DrawFlags flags){
			m_drawFlags = flags;
		}

		/// Get the drawing flags.
		public DrawFlags GetFlags(){
			return m_drawFlags;
		}
	
		/// Append flags to the current flags.
		public void AppendFlags(DrawFlags flags){
			m_drawFlags |= flags;
		}

		/// Clear flags from the current flags.
		public void ClearFlags(DrawFlags flags){
			m_drawFlags &= ~flags;
		}

		/// Draw a closed polygon provided in CCW order.
		public abstract void DrawPolygon(b2Vec2[] vertices, int vertexCount, Color color);

		/// Draw a solid closed polygon provided in CCW order.
		public abstract void DrawSolidPolygon(b2Vec2[] vertices, int vertexCount, Color color);

		/// Draw a circle.
		public abstract void DrawCircle(b2Vec2 center, float radius, Color color);
	
		/// Draw a solid circle.
		public abstract void DrawSolidCircle(b2Vec2 center, float radius, b2Vec2 axis, Color color);
	
		/// Draw a line segment.
		public abstract void DrawSegment(b2Vec2 p1, b2Vec2 p2, Color color);

		/// Draw a transform. Choose your own length scale.
		/// @param xf a transform.
		public abstract void DrawTransform(b2Transform xf);

	}
}
