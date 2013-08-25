using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// This class collides and edge and a polygon, taking into account edge adjacency.
	class EPCollider
	{
		// Algorithm:
		// 1. Classify v1 and v2
		// 2. Classify polygon centroid as front or back
		// 3. Flip normal if necessary
		// 4. Initialize normal range to [-pi, pi] about face normal
		// 5. Adjust normal range according to adjacent edges
		// 6. Visit each separating axes, only accept axes within the range
		// 7. Return if _any_ axis indicates separation
		// 8. Clip
		public void Collide(out Manifold manifold, EdgeShape edgeA, Transform xfA, PolygonShape polygonB, Transform xfB){
			manifold = new Manifold();
			m_xf = Utilities.MulT(xfA, xfB);
	
			m_centroidB = Utilities.Mul(m_xf, polygonB.m_centroid);
	
			m_v0 = edgeA.m_vertex0;
			m_v1 = edgeA.m_vertex1;
			m_v2 = edgeA.m_vertex2;
			m_v3 = edgeA.m_vertex3;
	
			bool hasVertex0 = edgeA.m_hasVertex0;
			bool hasVertex3 = edgeA.m_hasVertex3;
	
			Vec2 edge1 = m_v2 - m_v1;
			edge1.Normalize();
			m_normal1.Set(edge1.y, -edge1.x);
			float offset1 = Utilities.Dot(m_normal1, m_centroidB - m_v1);
			float offset0 = 0.0f, offset2 = 0.0f;
			bool convex1 = false, convex2 = false;
	
			// Is there a preceding edge?
			if (hasVertex0)
			{
				Vec2 edge0 = m_v1 - m_v0;
				edge0.Normalize();
				m_normal0.Set(edge0.y, -edge0.x);
				convex1 = Utilities.Cross(edge0, edge1) >= 0.0f;
				offset0 = Utilities.Dot(m_normal0, m_centroidB - m_v0);
			}
	
			// Is there a following edge?
			if (hasVertex3)
			{
				Vec2 edge2 = m_v3 - m_v2;
				edge2.Normalize();
				m_normal2.Set(edge2.y, -edge2.x);
				convex2 = Utilities.Cross(edge1, edge2) > 0.0f;
				offset2 = Utilities.Dot(m_normal2, m_centroidB - m_v2);
			}
	
			// Determine front or back collision. Determine collision normal limits.
			if (hasVertex0 && hasVertex3)
			{
				if (convex1 && convex2)
				{
					m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = m_normal0;
						m_upperLimit = m_normal2;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = -m_normal1;
						m_upperLimit = -m_normal1;
					}
				}
				else if (convex1)
				{
					m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = m_normal0;
						m_upperLimit = m_normal1;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = -m_normal2;
						m_upperLimit = -m_normal1;
					}
				}
				else if (convex2)
				{
					m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = m_normal1;
						m_upperLimit = m_normal2;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = -m_normal1;
						m_upperLimit = -m_normal0;
					}
				}
				else
				{
					m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = m_normal1;
						m_upperLimit = m_normal1;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = -m_normal2;
						m_upperLimit = -m_normal0;
					}
				}
			}
			else if (hasVertex0)
			{
				if (convex1)
				{
					m_front = offset0 >= 0.0f || offset1 >= 0.0f;
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = m_normal0;
						m_upperLimit = -m_normal1;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = m_normal1;
						m_upperLimit = -m_normal1;
					}
				}
				else
				{
					m_front = offset0 >= 0.0f && offset1 >= 0.0f;
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = m_normal1;
						m_upperLimit = -m_normal1;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = m_normal1;
						m_upperLimit = -m_normal0;
					}
				}
			}
			else if (hasVertex3)
			{
				if (convex2)
				{
					m_front = offset1 >= 0.0f || offset2 >= 0.0f;
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = -m_normal1;
						m_upperLimit = m_normal2;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = -m_normal1;
						m_upperLimit = m_normal1;
					}
				}
				else
				{
					m_front = offset1 >= 0.0f && offset2 >= 0.0f;
					if (m_front)
					{
						m_normal = m_normal1;
						m_lowerLimit = -m_normal1;
						m_upperLimit = m_normal1;
					}
					else
					{
						m_normal = -m_normal1;
						m_lowerLimit = -m_normal2;
						m_upperLimit = m_normal1;
					}
				}		
			}
			else
			{
				m_front = offset1 >= 0.0f;
				if (m_front)
				{
					m_normal = m_normal1;
					m_lowerLimit = -m_normal1;
					m_upperLimit = -m_normal1;
				}
				else
				{
					m_normal = -m_normal1;
					m_lowerLimit = m_normal1;
					m_upperLimit = m_normal1;
				}
			}
	
			// Get polygonB in frameA
			m_polygonB.count = polygonB.m_count;
			for (int i = 0; i < polygonB.m_count; ++i)
			{
				m_polygonB.vertices[i] = Utilities.Mul(m_xf, polygonB.m_vertices[i]);
				m_polygonB.normals[i] = Utilities.Mul(m_xf.q, polygonB.m_normals[i]);
			}
	
			m_radius = 2.0f * Settings._polygonRadius;

			manifold.points.Clear();
	
			EPAxis edgeAxis = ComputeEdgeSeparation();
	
			// If no valid normal can be found than this edge should not collide.
			if (edgeAxis.type == EPAxisType.e_unknown)
			{
				return;
			}
	
			if (edgeAxis.separation > m_radius)
			{
				return;
			}
	
			EPAxis polygonAxis = ComputePolygonSeparation();
			if (polygonAxis.type != EPAxisType.e_unknown && polygonAxis.separation > m_radius)
			{
				return;
			}
	
			// Use hysteresis for jitter reduction.
			const float k_relativeTol = 0.98f;
			const float k_absoluteTol = 0.001f;
	
			EPAxis primaryAxis;
			if (polygonAxis.type == EPAxisType.e_unknown)
			{
				primaryAxis = edgeAxis;
			}
			else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
			{
				primaryAxis = polygonAxis;
			}
			else
			{
				primaryAxis = edgeAxis;
			}
	
			ClipVertex[] ie = new ClipVertex[2];
			ReferenceFace rf = new ReferenceFace();
			if (primaryAxis.type == EPAxisType.e_edgeA)
			{
				manifold.type = Manifold.ManifoldType.e_faceA;
		
				// Search for the polygon normal that is most anti-parallel to the edge normal.
				int bestIndex = 0;
				float bestValue = Utilities.Dot(m_normal, m_polygonB.normals[0]);
				for (int i = 1; i < m_polygonB.count; ++i)
				{
					float value = Utilities.Dot(m_normal, m_polygonB.normals[i]);
					if (value < bestValue)
					{
						bestValue = value;
						bestIndex = i;
					}
				}
		
				int i1 = bestIndex;
				int i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;
		
				ie[0].v = m_polygonB.vertices[i1];
				ie[0].id.cf.indexA = 0;
				ie[0].id.cf.indexB = (byte)i1;
				ie[0].id.cf.typeA = ContactFeature.FeatureType.e_face;
				ie[0].id.cf.typeB = ContactFeature.FeatureType.e_vertex;
		
				ie[1].v = m_polygonB.vertices[i2];
				ie[1].id.cf.indexA = 0;
				ie[1].id.cf.indexB = (byte)i2;
				ie[1].id.cf.typeA = ContactFeature.FeatureType.e_face;
				ie[1].id.cf.typeB = ContactFeature.FeatureType.e_vertex;
		
				if (m_front)
				{
					rf.i1 = 0;
					rf.i2 = 1;
					rf.v1 = m_v1;
					rf.v2 = m_v2;
					rf.normal = m_normal1;
				}
				else
				{
					rf.i1 = 1;
					rf.i2 = 0;
					rf.v1 = m_v2;
					rf.v2 = m_v1;
					rf.normal = -m_normal1;
				}		
			}
			else
			{
				manifold.type = Manifold.ManifoldType.e_faceB;
		
				ie[0].v = m_v1;
				ie[0].id.cf.indexA = 0;
				ie[0].id.cf.indexB = (byte)primaryAxis.index;
				ie[0].id.cf.typeA = ContactFeature.FeatureType.e_vertex;
				ie[0].id.cf.typeB = ContactFeature.FeatureType.e_face;
		
				ie[1].v = m_v2;
				ie[1].id.cf.indexA = 0;
				ie[1].id.cf.indexB = (byte)primaryAxis.index;		
				ie[1].id.cf.typeA = ContactFeature.FeatureType.e_vertex;
				ie[1].id.cf.typeB = ContactFeature.FeatureType.e_face;
		
				rf.i1 = primaryAxis.index;
				rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
				rf.v1 = m_polygonB.vertices[rf.i1];
				rf.v2 = m_polygonB.vertices[rf.i2];
				rf.normal = m_polygonB.normals[rf.i1];
			}
	
			rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
			rf.sideNormal2 = -rf.sideNormal1;
			rf.sideOffset1 = Utilities.Dot(rf.sideNormal1, rf.v1);
			rf.sideOffset2 = Utilities.Dot(rf.sideNormal2, rf.v2);
	
			// Clip incident edge against extruded edge1 side edges.
			ClipVertex[] clipPoints1 = new ClipVertex[2];
			ClipVertex[] clipPoints2 = new ClipVertex[2];
			int np;
	
			// Clip to box side 1
			np = Collision.ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
	
			if (np < Settings._maxManifoldPoints)
			{
				return;
			}
	
			// Clip to negative box side 1
			np = Collision.ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
	
			if (np < Settings._maxManifoldPoints)
			{
				return;
			}
	
			// Now clipPoints2 contains the clipped points.
			if (primaryAxis.type == EPAxisType.e_edgeA)
			{
				manifold.localNormal = rf.normal;
				manifold.localPoint = rf.v1;
			}
			else
			{
				manifold.localNormal = polygonB.m_normals[rf.i1];
				manifold.localPoint = polygonB.m_vertices[rf.i1];
			}

			manifold.points.Clear();
			for (int i = 0; i < Settings._maxManifoldPoints; ++i)
			{
				float separation;
		
				separation = Utilities.Dot(rf.normal, clipPoints2[i].v - rf.v1);
		
				if (separation <= m_radius)
				{
					ManifoldPoint cp = new ManifoldPoint();
			
					if (primaryAxis.type == EPAxisType.e_edgeA)
					{
						cp.localPoint = Utilities.MulT(m_xf, clipPoints2[i].v);
						cp.id = clipPoints2[i].id;
					}
					else
					{
						cp.localPoint = clipPoints2[i].v;
						cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
						cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
						cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
						cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
					}

					manifold.points.Add(cp);
				}
			}
		}



		EPAxis ComputeEdgeSeparation(){
			EPAxis axis;
			axis.type = EPAxisType.e_edgeA;
			axis.index = m_front ? 0 : 1;
			axis.separation = Single.MaxValue;
	
			for (int i = 0; i < m_polygonB.count; ++i)
			{
				float s = Utilities.Dot(m_normal, m_polygonB.vertices[i] - m_v1);
				if (s < axis.separation)
				{
					axis.separation = s;
				}
			}
	
			return axis;
		}

		EPAxis ComputePolygonSeparation(){
			EPAxis axis;
			axis.type = EPAxisType.e_unknown;
			axis.index = -1;
			axis.separation = -Single.MaxValue;

			Vec2 perp = new Vec2(-m_normal.y, m_normal.x);

			for (int i = 0; i < m_polygonB.count; ++i)
			{
				Vec2 n = -m_polygonB.normals[i];
		
				float s1 = Utilities.Dot(n, m_polygonB.vertices[i] - m_v1);
				float s2 = Utilities.Dot(n, m_polygonB.vertices[i] - m_v2);
				float s = Math.Min(s1, s2);
		
				if (s > m_radius)
				{
					// No collision
					axis.type = EPAxisType.e_edgeB;
					axis.index = i;
					axis.separation = s;
					return axis;
				}
		
				// Adjacency
				if (Utilities.Dot(n, perp) >= 0.0f)
				{
					if (Utilities.Dot(n - m_upperLimit, m_normal) < -Settings._angularSlop)
					{
						continue;
					}
				}
				else
				{
					if (Utilities.Dot(n - m_lowerLimit, m_normal) < -Settings._angularSlop)
					{
						continue;
					}
				}
		
				if (s > axis.separation)
				{
					axis.type = EPAxisType.e_edgeB;
					axis.index = i;
					axis.separation = s;
				}
			}
	
			return axis;
		}
	
		enum VertexType
		{
			e_isolated,
			e_concave,
			e_convex
		};
	
		TempPolygon m_polygonB = new TempPolygon();
	
		Transform m_xf;
		Vec2 m_centroidB;
		Vec2 m_v0, m_v1, m_v2, m_v3;
		Vec2 m_normal0, m_normal1, m_normal2;
		Vec2 m_normal;
		VertexType m_type1, m_type2;
		Vec2 m_lowerLimit, m_upperLimit;
		float m_radius;
		bool m_front;
	}
}
