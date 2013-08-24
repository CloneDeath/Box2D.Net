using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenTK.Graphics.OpenGL;
using Box2D;
using System.Drawing;

namespace Testbed.Framework {
	class DebugDraw : b2Draw {
		public override void DrawCircle(b2Vec2 center, float radius, Color color) {
			float k_segments = 16.0f;
			float k_increment = 2.0f * (float)System.Math.PI / k_segments;
			float theta = 0.0f;
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < k_segments; ++i) {
				b2Vec2 v = center + radius * new b2Vec2((float)System.Math.Cos(theta), (float)System.Math.Sin(theta));
				GL.Vertex2(v.x, v.y);
				theta += k_increment;
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawPolygon(b2Vec2[] vertices, int vertexCount, Color color) {
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < vertexCount; ++i) {
				GL.Vertex2(vertices[i].x, vertices[i].y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawSegment(b2Vec2 p1, b2Vec2 p2, Color color) {
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.Lines);
			{
				GL.Vertex2(p1.x, p1.y);
				GL.Vertex2(p2.x, p2.y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawSolidCircle(b2Vec2 center, float radius, b2Vec2 axis, Color color) {
			float k_segments = 16.0f;
			float k_increment = 2.0f * (float)System.Math.PI / k_segments;
			float theta = 0.0f;
			GL.Color3(0.5f * color.R, 0.5f * color.G, 0.5f * color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.TriangleFan);
			for (int i = 0; i < k_segments; ++i) {
				b2Vec2 v = center + radius * new b2Vec2((float)System.Math.Cos(theta), (float)System.Math.Sin(theta));
				GL.Vertex2(v.x, v.y);
				theta += k_increment;
			}
			GL.End();

			theta = 0.0f;
			GL.Color4(color.R, color.G, color.B, 1.0f);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < k_segments; ++i) {
				b2Vec2 v = center + radius * new b2Vec2((float)System.Math.Cos(theta), (float)System.Math.Sin(theta));
				GL.Vertex2(v.x, v.y);
				theta += k_increment;
			}
			GL.End();

			b2Vec2 p = center + radius * axis;
			GL.Begin(BeginMode.Lines);
			GL.Vertex2(center.x, center.y);
			GL.Vertex2(p.x, p.y);
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawSolidPolygon(b2Vec2[] vertices, int vertexCount, Color color) {
			GL.Color3(0.5f * color.R, 0.5f * color.G, 0.5f * color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.TriangleFan);
			for (int i = 0; i < vertexCount; ++i) {
				GL.Vertex2(vertices[i].x, vertices[i].y);
			}
			GL.End();

			GL.Color4(color.R, color.G, color.B, 1.0f);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < vertexCount; ++i) {
				GL.Vertex2(vertices[i].x, vertices[i].y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawTransform(b2Transform xf) {
			b2Vec2 p1 = xf.p, p2;
			float k_axisScale = 0.4f;
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.Lines);
			{
				GL.Color3(1.0f, 0.0f, 0.0f);
				GL.Vertex2(p1.x, p1.y);
				p2 = p1 + k_axisScale * xf.q.GetXAxis();
				GL.Vertex2(p2.x, p2.y);

				GL.Color3(0.0f, 1.0f, 0.0f);
				GL.Vertex2(p1.x, p1.y);
				p2 = p1 + k_axisScale * xf.q.GetYAxis();
				GL.Vertex2(p2.x, p2.y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public void DrawSegment(b2Vec2 p1, b2Vec2 p2, Color color, params object[] p) {
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.Lines);
			{
				GL.Vertex2(p1.x, p1.y);
				GL.Vertex2(p2.x, p2.y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public void DrawString(string title, params object[] args) {
			Console.WriteLine(String.Format(title, args));
		}

		internal void DrawPoint(b2Vec2 xy, float p, Color c) {
			GL.Begin(BeginMode.Points);
			{
				GL.Color3(c);
				GL.Vertex2(xy.x, xy.y);
			}
			GL.End();
		}
	}
}
