using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class b2ContactSolver
	{
		public b2ContactSolver(b2ContactSolverDef def) {
			m_step = def.step;
			m_positionConstraints = new List<b2ContactPositionConstraint>();
			m_velocityConstraints = new List<b2ContactVelocityConstraint>();
			m_positions = def.positions;
			m_velocities = def.velocities;
			m_contacts = def.contacts;

			// Initialize position independent portions of the constraints.
			for (int i = 0; i < def.contacts.Count(); ++i) {
				b2Contact contact = m_contacts[i];

				b2Fixture fixtureA = contact.m_fixtureA;
				b2Fixture fixtureB = contact.m_fixtureB;
				b2Shape shapeA = fixtureA.GetShape();
				b2Shape shapeB = fixtureB.GetShape();
				float radiusA = shapeA.m_radius;
				float radiusB = shapeB.m_radius;
				b2Body bodyA = fixtureA.GetBody();
				b2Body bodyB = fixtureB.GetBody();
				b2Manifold manifold = contact.GetManifold();

				int pointCount = manifold.points.Count();
				Utilities.Assert(pointCount > 0);

				b2ContactVelocityConstraint vc = new b2ContactVelocityConstraint();
				vc.friction = contact.m_friction;
				vc.restitution = contact.m_restitution;
				vc.tangentSpeed = contact.m_tangentSpeed;
				vc.indexA = bodyA.m_islandIndex;
				vc.indexB = bodyB.m_islandIndex;
				vc.invMassA = bodyA.m_invMass;
				vc.invMassB = bodyB.m_invMass;
				vc.invIA = bodyA.m_invI;
				vc.invIB = bodyB.m_invI;
				vc.contactIndex = i;
				//vc.points.Count() = pointCount;
				vc.K.SetZero();
				vc.normalMass.SetZero();

				b2ContactPositionConstraint pc = new b2ContactPositionConstraint();
				pc.indexA = bodyA.m_islandIndex;
				pc.indexB = bodyB.m_islandIndex;
				pc.invMassA = bodyA.m_invMass;
				pc.invMassB = bodyB.m_invMass;
				pc.localCenterA = bodyA.m_sweep.localCenter;
				pc.localCenterB = bodyB.m_sweep.localCenter;
				pc.invIA = bodyA.m_invI;
				pc.invIB = bodyB.m_invI;
				pc.localNormal = manifold.localNormal;
				pc.localPoint = manifold.localPoint;
				pc.pointCount = pointCount;
				pc.radiusA = radiusA;
				pc.radiusB = radiusB;
				pc.type = manifold.type;

				for (int j = 0; j < pointCount; ++j) {
					b2ManifoldPoint cp = manifold.points[j];
					b2VelocityConstraintPoint vcp = new b2VelocityConstraintPoint();
					
					if (m_step.warmStarting) {
						vcp.normalImpulse = m_step.dtRatio * cp.normalImpulse;
						vcp.tangentImpulse = m_step.dtRatio * cp.tangentImpulse;
					} else {
						vcp.normalImpulse = 0.0f;
						vcp.tangentImpulse = 0.0f;
					}

					vcp.rA.SetZero();
					vcp.rB.SetZero();
					vcp.normalMass = 0.0f;
					vcp.tangentMass = 0.0f;
					vcp.velocityBias = 0.0f;
					vc.points.Add(vcp);

					pc.localPoints[j] = cp.localPoint;
				}
				m_velocityConstraints.Add(vc);
				m_positionConstraints.Add(pc);
			}
		}

		public void InitializeVelocityConstraints() {
			for (int i = 0; i < m_contacts.Count(); ++i) {
				b2ContactVelocityConstraint vc = m_velocityConstraints[i];
				b2ContactPositionConstraint pc = m_positionConstraints[i];

				float radiusA = pc.radiusA;
				float radiusB = pc.radiusB;
				b2Manifold manifold = m_contacts[vc.contactIndex].GetManifold();

				int indexA = vc.indexA;
				int indexB = vc.indexB;

				float mA = vc.invMassA;
				float mB = vc.invMassB;
				float iA = vc.invIA;
				float iB = vc.invIB;
				b2Vec2 localCenterA = pc.localCenterA;
				b2Vec2 localCenterB = pc.localCenterB;

				b2Vec2 cA = m_positions[indexA].c;
				float aA = m_positions[indexA].a;
				b2Vec2 vA = m_velocities[indexA].v;
				float wA = m_velocities[indexA].w;

				b2Vec2 cB = m_positions[indexB].c;
				float aB = m_positions[indexB].a;
				b2Vec2 vB = m_velocities[indexB].v;
				float wB = m_velocities[indexB].w;

				Utilities.Assert(manifold.points.Count() > 0);

				b2Transform xfA = new b2Transform();
				b2Transform xfB = new b2Transform();
				xfA.q.Set(aA);
				xfB.q.Set(aB);
				xfA.p = cA - Utilities.b2Mul(xfA.q, localCenterA);
				xfB.p = cB - Utilities.b2Mul(xfB.q, localCenterB);

				b2WorldManifold worldManifold = new b2WorldManifold();
				worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

				vc.normal = worldManifold.normal;

				int pointCount = vc.points.Count;
				for (int j = 0; j < pointCount; ++j) {
					b2VelocityConstraintPoint vcp = vc.points[j];

					vcp.rA = worldManifold.points[j] - cA;
					vcp.rB = worldManifold.points[j] - cB;

					float rnA = Utilities.b2Cross(vcp.rA, vc.normal);
					float rnB = Utilities.b2Cross(vcp.rB, vc.normal);

					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					vcp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					b2Vec2 tangent = Utilities.b2Cross(vc.normal, 1.0f);

					float rtA = Utilities.b2Cross(vcp.rA, tangent);
					float rtB = Utilities.b2Cross(vcp.rB, tangent);

					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

					vcp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// Setup a velocity bias for restitution.
					vcp.velocityBias = 0.0f;
					float vRel = Utilities.b2Dot(vc.normal, vB + Utilities.b2Cross(wB, vcp.rB) - vA - Utilities.b2Cross(wA, vcp.rA));
					if (vRel < -b2Settings.b2_velocityThreshold) {
						vcp.velocityBias = -vc.restitution * vRel;
					}
				}

				// If we have two points, then prepare the block solver.
				if (vc.points.Count() == 2) {
					b2VelocityConstraintPoint vcp1 = vc.points[0];
					b2VelocityConstraintPoint vcp2 = vc.points[1];

					float rn1A = Utilities.b2Cross(vcp1.rA, vc.normal);
					float rn1B = Utilities.b2Cross(vcp1.rB, vc.normal);
					float rn2A = Utilities.b2Cross(vcp2.rA, vc.normal);
					float rn2B = Utilities.b2Cross(vcp2.rB, vc.normal);

					float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
					float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
					float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

					// Ensure a reasonable condition number.
					const float k_maxConditionNumber = 1000.0f;
					if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
						// K is safe to invert.
						vc.K.ex.Set(k11, k12);
						vc.K.ey.Set(k12, k22);
						vc.normalMass = vc.K.GetInverse();
					} else {
						// The constraints are redundant, just use one.
						// TODO_ERIN use deepest?
						vc.points.Clear();
						vc.points.Add(new b2VelocityConstraintPoint());
					}
				}
			}
		}

		public void WarmStart() {
			// Warm start.
			for (int i = 0; i < this.m_contacts.Count(); ++i) {
				b2ContactVelocityConstraint vc = m_velocityConstraints[i];

				int indexA = vc.indexA;
				int indexB = vc.indexB;
				float mA = vc.invMassA;
				float iA = vc.invIA;
				float mB = vc.invMassB;
				float iB = vc.invIB;
				int pointCount = vc.points.Count();

				b2Vec2 vA = m_velocities[indexA].v;
				float wA = m_velocities[indexA].w;
				b2Vec2 vB = m_velocities[indexB].v;
				float wB = m_velocities[indexB].w;

				b2Vec2 normal = vc.normal;
				b2Vec2 tangent = Utilities.b2Cross(normal, 1.0f);

				for (int j = 0; j < pointCount; ++j) {
					b2VelocityConstraintPoint vcp = vc.points[j];
					b2Vec2 P = vcp.normalImpulse * normal + vcp.tangentImpulse * tangent;
					wA -= iA * Utilities.b2Cross(vcp.rA, P);
					vA -= mA * P;
					wB += iB * Utilities.b2Cross(vcp.rB, P);
					vB += mB * P;
				}

				m_velocities[indexA].v = vA;
				m_velocities[indexA].w = wA;
				m_velocities[indexB].v = vB;
				m_velocities[indexB].w = wB;
			}
		}

		public void SolveVelocityConstraints(){
			for (int i = 0; i < m_contacts.Count(); ++i) {
				b2ContactVelocityConstraint vc = m_velocityConstraints[i];

				int indexA = vc.indexA;
				int indexB = vc.indexB;
				float mA = vc.invMassA;
				float iA = vc.invIA;
				float mB = vc.invMassB;
				float iB = vc.invIB;
				int pointCount = vc.points.Count();

				b2Vec2 vA = m_velocities[indexA].v;
				float wA = m_velocities[indexA].w;
				b2Vec2 vB = m_velocities[indexB].v;
				float wB = m_velocities[indexB].w;

				b2Vec2 normal = vc.normal;
				b2Vec2 tangent = Utilities.b2Cross(normal, 1.0f);
				float friction = vc.friction;

				Utilities.Assert(pointCount == 1 || pointCount == 2);

				// Solve tangent constraints first because non-penetration is more important
				// than friction.
				for (int j = 0; j < pointCount; ++j) {
					b2VelocityConstraintPoint vcp = vc.points[j];

					// Relative velocity at contact
					b2Vec2 dv = vB + Utilities.b2Cross(wB, vcp.rB) - vA - Utilities.b2Cross(wA, vcp.rA);

					// Compute tangent force
					float vt = Utilities.b2Dot(dv, tangent) - vc.tangentSpeed;
					float lambda = vcp.tangentMass * (-vt);

					// b2Clamp the accumulated force
					float maxFriction = friction * vcp.normalImpulse;
					float newImpulse = Utilities.b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
					lambda = newImpulse - vcp.tangentImpulse;
					vcp.tangentImpulse = newImpulse;

					// Apply contact impulse
					b2Vec2 P = lambda * tangent;

					vA -= mA * P;
					wA -= iA * Utilities.b2Cross(vcp.rA, P);

					vB += mB * P;
					wB += iB * Utilities.b2Cross(vcp.rB, P);
				}

				// Solve normal constraints
				if (vc.points.Count() == 1) {
					b2VelocityConstraintPoint vcp = vc.points[0];

					// Relative velocity at contact
					b2Vec2 dv = vB + Utilities.b2Cross(wB, vcp.rB) - vA - Utilities.b2Cross(wA, vcp.rA);

					// Compute normal impulse
					float vn = Utilities.b2Dot(dv, normal);
					float lambda = -vcp.normalMass * (vn - vcp.velocityBias);

					// b2Clamp the accumulated impulse
					float newImpulse = Math.Max(vcp.normalImpulse + lambda, 0.0f);
					lambda = newImpulse - vcp.normalImpulse;
					vcp.normalImpulse = newImpulse;

					// Apply contact impulse
					b2Vec2 P = lambda * normal;
					vA -= mA * P;
					wA -= iA * Utilities.b2Cross(vcp.rA, P);

					vB += mB * P;
					wB += iB * Utilities.b2Cross(vcp.rB, P);
				} else {
					// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
					// Build the mini LCP for this contact patch
					//
					// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
					//
					// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
					// b = vn0 - velocityBias
					//
					// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
					// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
					// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
					// solution that satisfies the problem is chosen.
					// 
					// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
					// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
					//
					// Substitute:
					// 
					// x = a + d
					// 
					// a := old total impulse
					// x := new total impulse
					// d := incremental impulse 
					//
					// For the current iteration we extend the formula for the incremental impulse
					// to compute the new total impulse:
					//
					// vn = A * d + b
					//    = A * (x - a) + b
					//    = A * x + b - A * a
					//    = A * x + b'
					// b' = b - A * a;

					b2VelocityConstraintPoint cp1 = vc.points[0];
					b2VelocityConstraintPoint cp2 = vc.points[1];

					b2Vec2 a = new b2Vec2(cp1.normalImpulse, cp2.normalImpulse);
					Utilities.Assert(a.x >= 0.0f && a.y >= 0.0f);

					// Relative velocity at contact
					b2Vec2 dv1 = vB + Utilities.b2Cross(wB, cp1.rB) - vA - Utilities.b2Cross(wA, cp1.rA);
					b2Vec2 dv2 = vB + Utilities.b2Cross(wB, cp2.rB) - vA - Utilities.b2Cross(wA, cp2.rA);

					// Compute normal velocity
					float vn1 = Utilities.b2Dot(dv1, normal);
					float vn2 = Utilities.b2Dot(dv2, normal);

					b2Vec2 b;
					b.x = vn1 - cp1.velocityBias;
					b.y = vn2 - cp2.velocityBias;

					// Compute b'
					b -= Utilities.b2Mul(vc.K, a);

					const float k_errorTol = 1e-3f;

					for (; ; ) {
						//
						// Case 1: vn = 0
						//
						// 0 = A * x + b'
						//
						// Solve for x:
						//
						// x = - inv(A) * b'
						//
						b2Vec2 x = -Utilities.b2Mul(vc.normalMass, b);

						if (x.x >= 0.0f && x.y >= 0.0f) {
							// Get the incremental impulse
							b2Vec2 d = x - a;

							// Apply incremental impulse
							b2Vec2 P1 = d.x * normal;
							b2Vec2 P2 = d.y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Utilities.b2Cross(cp1.rA, P1) + Utilities.b2Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Utilities.b2Cross(cp1.rB, P1) + Utilities.b2Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.x;
							cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER
		                    // Postconditions
		                    dv1 = vB + Utilities.b2Cross(wB, cp1.rB) - vA - Utilities.b2Cross(wA, cp1.rA);
		                    dv2 = vB + Utilities.b2Cross(wB, cp2.rB) - vA - Utilities.b2Cross(wA, cp2.rA);

		                    // Compute normal velocity
		                    vn1 = Utilities.b2Dot(dv1, normal);
		                    vn2 = Utilities.b2Dot(dv2, normal);

		                    Utilities.Assert(Math.Abs(vn1 - cp1.velocityBias) < k_errorTol);
		                    Utilities.Assert(Math.Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
							break;
						}

						//
						// Case 2: vn1 = 0 and x2 = 0
						//
						//   0 = a11 * x1 + a12 * 0 + b1' 
						// vn2 = a21 * x1 + a22 * 0 + b2'
						//
						x.x = -cp1.normalMass * b.x;
						x.y = 0.0f;
						vn1 = 0.0f;
						vn2 = vc.K.ex.y * x.x + b.y;

						if (x.x >= 0.0f && vn2 >= 0.0f) {
							// Get the incremental impulse
							b2Vec2 d = x - a;

							// Apply incremental impulse
							b2Vec2 P1 = d.x * normal;
							b2Vec2 P2 = d.y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Utilities.b2Cross(cp1.rA, P1) + Utilities.b2Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Utilities.b2Cross(cp1.rB, P1) + Utilities.b2Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.x;
							cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER
		                    // Postconditions
		                    dv1 = vB + Utilities.b2Cross(wB, cp1.rB) - vA - Utilities.b2Cross(wA, cp1.rA);

		                    // Compute normal velocity
		                    vn1 = Utilities.b2Dot(dv1, normal);

		                    Utilities.Assert(Math.Abs(vn1 - cp1.velocityBias) < k_errorTol);
#endif
							break;
						}


						//
						// Case 3: vn2 = 0 and x1 = 0
						//
						// vn1 = a11 * 0 + a12 * x2 + b1' 
						//   0 = a21 * 0 + a22 * x2 + b2'
						//
						x.x = 0.0f;
						x.y = -cp2.normalMass * b.y;
						vn1 = vc.K.ey.x * x.y + b.x;
						vn2 = 0.0f;

						if (x.y >= 0.0f && vn1 >= 0.0f) {
							// Resubstitute for the incremental impulse
							b2Vec2 d = x - a;

							// Apply incremental impulse
							b2Vec2 P1 = d.x * normal;
							b2Vec2 P2 = d.y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Utilities.b2Cross(cp1.rA, P1) + Utilities.b2Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Utilities.b2Cross(cp1.rB, P1) + Utilities.b2Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.x;
							cp2.normalImpulse = x.y;

#if B2_DEBUG_SOLVER
		                    // Postconditions
		                    dv2 = vB + Utilities.b2Cross(wB, cp2.rB) - vA - Utilities.b2Cross(wA, cp2.rA);

		                    // Compute normal velocity
		                    vn2 = Utilities.b2Dot(dv2, normal);

		                    Utilities.Assert(Math.Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
							break;
						}

						//
						// Case 4: x1 = 0 and x2 = 0
						// 
						// vn1 = b1
						// vn2 = b2;
						x.x = 0.0f;
						x.y = 0.0f;
						vn1 = b.x;
						vn2 = b.y;

						if (vn1 >= 0.0f && vn2 >= 0.0f) {
							// Resubstitute for the incremental impulse
							b2Vec2 d = x - a;

							// Apply incremental impulse
							b2Vec2 P1 = d.x * normal;
							b2Vec2 P2 = d.y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Utilities.b2Cross(cp1.rA, P1) + Utilities.b2Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Utilities.b2Cross(cp1.rB, P1) + Utilities.b2Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.x;
							cp2.normalImpulse = x.y;

							break;
						}

						// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
						break;
					}
				}

				var velA = m_velocities[indexA];
				velA.v = vA;
				velA.w = wA;
				m_velocities[indexA] = velA;

				var velB = m_velocities[indexB];
				velB.v = vB;
				velB.w = wB;
				m_velocities[indexB] = velB;
			}
		}
		public void StoreImpulses(){
			for (int i = 0; i < m_contacts.Count(); ++i) {
				b2ContactVelocityConstraint vc = m_velocityConstraints[i];
				b2Manifold manifold = m_contacts[vc.contactIndex].GetManifold();

				for (int j = 0; j < vc.points.Count(); ++j) {
					manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
					manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
				}
			}
		}

		public bool SolvePositionConstraints(){
			float minSeparation = 0.0f;

			for (int i = 0; i < m_contacts.Count(); ++i) {
				b2ContactPositionConstraint pc = m_positionConstraints[i];

				int indexA = pc.indexA;
				int indexB = pc.indexB;
				b2Vec2 localCenterA = pc.localCenterA;
				float mA = pc.invMassA;
				float iA = pc.invIA;
				b2Vec2 localCenterB = pc.localCenterB;
				float mB = pc.invMassB;
				float iB = pc.invIB;
				int pointCount = pc.pointCount;

				b2Vec2 cA = m_positions[indexA].c;
				float aA = m_positions[indexA].a;

				b2Vec2 cB = m_positions[indexB].c;
				float aB = m_positions[indexB].a;

				// Solve normal constraints
				for (int j = 0; j < pointCount; ++j) {
					b2Transform xfA = new b2Transform();
					b2Transform xfB = new b2Transform();
					xfA.q.Set(aA);
					xfB.q.Set(aB);
					xfA.p = cA - Utilities.b2Mul(xfA.q, localCenterA);
					xfB.p = cB - Utilities.b2Mul(xfB.q, localCenterB);

					b2PositionSolverManifold psm = new b2PositionSolverManifold();
					psm.Initialize(pc, xfA, xfB, j);
					b2Vec2 normal = psm.normal;

					b2Vec2 point = psm.point;
					float separation = psm.separation;

					b2Vec2 rA = point - cA;
					b2Vec2 rB = point - cB;

					// Track max constraint error.
					minSeparation = Math.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					float C = Utilities.b2Clamp(b2Settings.b2_baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0f);

					// Compute the effective mass.
					float rnA = Utilities.b2Cross(rA, normal);
					float rnB = Utilities.b2Cross(rB, normal);
					float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					// Compute normal impulse
					float impulse = K > 0.0f ? -C / K : 0.0f;

					b2Vec2 P = impulse * normal;

					cA -= mA * P;
					aA -= iA * Utilities.b2Cross(rA, P);

					cB += mB * P;
					aB += iB * Utilities.b2Cross(rB, P);
				}

				m_positions[indexA].c = cA;
				m_positions[indexA].a = aA;

				m_positions[indexB].c = cB;
				m_positions[indexB].a = aB;
			}

			// We can't expect minSpeparation >= -b2_linearSlop because we don't
			// push the separation above -b2_linearSlop.
			return minSeparation >= -3.0f * b2Settings.b2_linearSlop;
		}
		public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB){
			throw new NotImplementedException();
			//float minSeparation = 0.0f;

			//for (int i = 0; i < m_contacts.Count(); ++i)
			//{
			//    b2ContactPositionConstraint* pc = m_positionConstraints + i;

			//    int indexA = pc.indexA;
			//    int indexB = pc.indexB;
			//    b2Vec2 localCenterA = pc.localCenterA;
			//    b2Vec2 localCenterB = pc.localCenterB;
			//    int pointCount = pc.pointCount;

			//    float mA = 0.0f;
			//    float iA = 0.0f;
			//    if (indexA == toiIndexA || indexA == toiIndexB)
			//    {
			//        mA = pc.invMassA;
			//        iA = pc.invIA;
			//    }

			//    float mB = 0.0f;
			//    float iB = 0.0f;
			//    if (indexB == toiIndexA || indexB == toiIndexB)
			//    {
			//        mB = pc.invMassB;
			//        iB = pc.invIB;
			//    }

			//    b2Vec2 cA = m_positions[indexA].c;
			//    float aA = m_positions[indexA].a;

			//    b2Vec2 cB = m_positions[indexB].c;
			//    float aB = m_positions[indexB].a;

			//    // Solve normal constraints
			//    for (int j = 0; j < pointCount; ++j)
			//    {
			//        b2Transform xfA, xfB;
			//        xfA.q.Set(aA);
			//        xfB.q.Set(aB);
			//        xfA.p = cA - Utilities.b2Mul(xfA.q, localCenterA);
			//        xfB.p = cB - Utilities.b2Mul(xfB.q, localCenterB);

			//        b2PositionSolverManifold psm;
			//        psm.Initialize(pc, xfA, xfB, j);
			//        b2Vec2 normal = psm.normal;

			//        b2Vec2 point = psm.point;
			//        float separation = psm.separation;

			//        b2Vec2 rA = point - cA;
			//        b2Vec2 rB = point - cB;

			//        // Track max constraint error.
			//        minSeparation = Math.Min(minSeparation, separation);

			//        // Prevent large corrections and allow slop.
			//        float C = b2Clamp(b2_toiBaugarte * (separation +b2Settings.b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			//        // Compute the effective mass.
			//        float rnA = Utilities.b2Cross(rA, normal);
			//        float rnB = Utilities.b2Cross(rB, normal);
			//        float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			//        // Compute normal impulse
			//        float impulse = K > 0.0f ? - C / K : 0.0f;

			//        b2Vec2 P = impulse * normal;

			//        cA -= mA * P;
			//        aA -= iA * Utilities.b2Cross(rA, P);

			//        cB += mB * P;
			//        aB += iB * Utilities.b2Cross(rB, P);
			//    }

			//    m_positions[indexA].c = cA;
			//    m_positions[indexA].a = aA;

			//    m_positions[indexB].c = cB;
			//    m_positions[indexB].a = aB;
			//}

			//// We can't expect minSpeparation >= -b2_linearSlop because we don't
			//// push the separation above -b2_linearSlop.
			//return minSeparation >= -1.5f *b2Settings.b2_linearSlop;
		}

		public b2TimeStep m_step;
		public List<b2Position> m_positions;
		public List<b2Velocity> m_velocities;
		public List<b2ContactPositionConstraint> m_positionConstraints;
		public List<b2ContactVelocityConstraint> m_velocityConstraints;
		public List<b2Contact> m_contacts; //pointer to pointers
		//public int m_contacts.Count();
	};
}
