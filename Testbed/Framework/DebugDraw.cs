﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using OpenTK.Graphics.OpenGL;
using Box2D;
using System.Drawing;

namespace Testbed.Framework {
	class DebugDraw : Draw {
		public override void DrawCircle(Vec2 center, float radius, Color color) {
			float k_segments = 16.0f;
			float k_increment = 2.0f * (float)System.Math.PI / k_segments;
			float theta = 0.0f;
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < k_segments; ++i) {
				Vec2 v = center + radius * new Vec2((float)System.Math.Cos(theta), (float)System.Math.Sin(theta));
				GL.Vertex2(v.X, v.Y);
				theta += k_increment;
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawPolygon(Vec2[] vertices, int vertexCount, Color color) {
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < vertexCount; ++i) {
				GL.Vertex2(vertices[i].X, vertices[i].Y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawSegment(Vec2 p1, Vec2 p2, Color color) {
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.Lines);
			{
				GL.Vertex2(p1.X, p1.Y);
				GL.Vertex2(p2.X, p2.Y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color) {
			float k_segments = 16.0f;
			float k_increment = 2.0f * (float)System.Math.PI / k_segments;
			float theta = 0.0f;
			GL.Color3(0.5f * color.R, 0.5f * color.G, 0.5f * color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.TriangleFan);
			for (int i = 0; i < k_segments; ++i) {
				Vec2 v = center + radius * new Vec2((float)System.Math.Cos(theta), (float)System.Math.Sin(theta));
				GL.Vertex2(v.X, v.Y);
				theta += k_increment;
			}
			GL.End();

			theta = 0.0f;
			GL.Color4(color.R, color.G, color.B, 1.0f);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < k_segments; ++i) {
				Vec2 v = center + radius * new Vec2((float)System.Math.Cos(theta), (float)System.Math.Sin(theta));
				GL.Vertex2(v.X, v.Y);
				theta += k_increment;
			}
			GL.End();

			Vec2 p = center + radius * axis;
			GL.Begin(BeginMode.Lines);
			GL.Vertex2(center.X, center.Y);
			GL.Vertex2(p.X, p.Y);
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color) {
			GL.Color3(0.5f * color.R, 0.5f * color.G, 0.5f * color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.TriangleFan);
			for (int i = 0; i < vertexCount; ++i) {
				GL.Vertex2(vertices[i].X, vertices[i].Y);
			}
			GL.End();

			GL.Color4(color.R, color.G, color.B, 1.0f);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < vertexCount; ++i) {
				GL.Vertex2(vertices[i].X, vertices[i].Y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public override void DrawTransform(Transform xf) {
			Vec2 p1 = xf.p, p2;
			float k_axisScale = 0.4f;
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.Lines);
			{
				GL.Color3(1.0f, 0.0f, 0.0f);
				GL.Vertex2(p1.X, p1.Y);
				p2 = p1 + k_axisScale * xf.q.GetXAxis();
				GL.Vertex2(p2.X, p2.Y);

				GL.Color3(0.0f, 1.0f, 0.0f);
				GL.Vertex2(p1.X, p1.Y);
				p2 = p1 + k_axisScale * xf.q.GetYAxis();
				GL.Vertex2(p2.X, p2.Y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public void DrawSegment(Vec2 p1, Vec2 p2, Color color, params object[] p) {
			GL.Color3(color.R, color.G, color.B);
			GL.Disable(EnableCap.Texture2D);
			GL.Begin(BeginMode.Lines);
			{
				GL.Vertex2(p1.X, p1.Y);
				GL.Vertex2(p2.X, p2.Y);
			}
			GL.End();
			GL.Enable(EnableCap.Texture2D);
		}

		public void DrawString(string title, params object[] args) {
			Console.WriteLine(String.Format(title, args));
		}

		internal void DrawPoint(Vec2 xy, float p, Color c) {
			GL.Begin(BeginMode.Points);
			{
				GL.Color3(c);
				GL.Vertex2(xy.X, xy.Y);
			}
			GL.End();
		}

		internal void DrawAABB(AABB AABB, Color c) {
			this.DrawPolygon(new Vec2[] { 
				AABB.lowerBound,
				new Vec2(AABB.lowerBound.X, AABB.upperBound.Y),
				AABB.upperBound,
				new Vec2(AABB.upperBound.X, AABB.lowerBound.Y)}, 
				4, c);
		}
	}
}
