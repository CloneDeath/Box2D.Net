using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/*
	Position Correction Notes
	=========================
	I tried the several algorithms for position correction of the 2D revolute joint.
	I looked at these systems:
	- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
	- suspension bridge with 30 1m long planks of length 1m.
	- multi-link chain with 30 1m long links.

	Here are the algorithms:

	Baumgarte - A fraction of the position error is added to the velocity error. There is no
	separate position solver.

	Pseudo Velocities - After the velocity solver and position integration,
	the position error, Jacobian, and effective mass are recomputed. Then
	the velocity constraints are solved with pseudo velocities and a fraction
	of the position error is added to the pseudo velocity error. The pseudo
	velocities are initialized to zero and there is no warm-starting. After
	the position solver, the pseudo velocities are added to the positions.
	This is also called the First Order World method or the Position LCP method.

	Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
	position error is re-computed for each constraint and the positions are updated
	after the constraint is solved. The radius vectors (aka Jacobians) are
	re-computed too (otherwise the algorithm has horrible instability). The pseudo
	velocity states are not needed because they are effectively zero at the beginning
	of each iteration. Since we have the current position error, we allow the
	iterations to terminate early if the error becomes smaller thanSettings._linearSlop.

	Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
	each time a constraint is solved.

	Here are the results:
	Baumgarte - this is the cheapest algorithm but it has some stability problems,
	especially with the bridge. The chain links separate easily close to the root
	and they jitter as they struggle to pull together. This is one of the most common
	methods in the field. The big drawback is that the position correction artificially
	affects the momentum, thus leading to instabilities and false bounce. I used a
	bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
	factor makes joints and contacts more spongy.

	Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
	stable. However, joints still separate with large angular velocities. Drag the
	simple pendulum in a circle quickly and the joint will separate. The chain separates
	easily and does not recover. I used a bias factor of 0.2. A larger value lead to
	the bridge collapsing when a heavy cube drops on it.

	Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
	Velocities, but in other ways it is worse. The bridge and chain are much more
	stable, but the simple pendulum goes unstable at high angular velocities.

	Full NGS - stable in all tests. The joints display good stiffness. The bridge
	still sags, but this is better than infinite forces.

	Recommendations
	Pseudo Velocities are not really worthwhile because the bridge and chain cannot
	recover from joint separation. In other cases the benefit over Baumgarte is small.

	Modified NGS is not a robust method for the revolute joint due to the violent
	instability seen in the simple pendulum. Perhaps it is viable with other constraint
	types, especially scalar constraints where the effective mass is a scalar.

	This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
	and is very fast. I don't think we can escape Baumgarte, especially in highly
	demanding cases where high constraint fidelity is not needed.

	Full NGS is robust and easy on the eyes. I recommend this as an option for
	higher fidelity simulation and certainly for suspension bridges and long chains.
	Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
	joint separation can be problematic. The number of NGS iterations can be reduced
	for better performance without harming robustness much.

	Each joint in a can be handled differently in the position solver. So I recommend
	a system where the user can select the algorithm on a per joint basis. I would
	probably default to the slower Full NGS and let the user select the faster
	Baumgarte method in performance critical scenarios.
	*/

	/*
	Cache Performance

	The Box2D solvers are dominated by cache misses. Data structures are designed
	to increase the number of cache hits. Much of misses are due to random access
	to body data. The constraint structures are iterated over linearly, which leads
	to few cache misses.

	The bodies are not accessed during iteration. Instead read only data, such as
	the mass values are stored with the constraints. The mutable data are the constraint
	impulses and the bodies velocities/positions. The impulses are held inside the
	constraint structures. The body velocities/positions are held in compact, temporary
	arrays to increase the number of cache hits. Linear and angular velocity are
	stored in a single array since multiple arrays lead to multiple misses.
	*/

	/*
	2D Rotation

	R = [cos(theta) -sin(theta)]
		[sin(theta) cos(theta) ]

	thetaDot = omega

	Let q1 = cos(theta), q2 = sin(theta).
	R = [q1 -q2]
		[q2  q1]

	q1Dot = -thetaDot * q2
	q2Dot = thetaDot * q1

	q1_new = q1_old - dt * w * q2
	q2_new = q2_old + dt * w * q1
	then normalize.

	This might be faster than computing sin+cos.
	However, we can compute sin+cos of the same angle fast.
	*/
	internal class Island
	{
		public Island(ContactListener listener){
			m_listener = listener;

			m_bodies = new List<Body>();
			m_contacts = new List<Contact>();
			m_joints = new List<Joint>();

			m_velocities = new List<Velocity>();
			m_positions = new List<Position>();
		}


		public void Clear()
		{
			m_bodies.Clear();
			m_contacts.Clear();
			m_joints.Clear();
		}

		public void Solve(Profile profile, TimeStep step, Vec2 gravity, bool allowSleep) {
			Timer timer = new Timer();

			float h = step.dt;

			// Integrate velocities and apply damping. Initialize the body state.
			for (int i = 0; i < m_bodies.Count(); i++)
			{
				Body b = m_bodies[i];
			    Vec2 c = b.m_sweep.c;
			    float a = b.m_sweep.a;
			    Vec2 v = b.m_linearVelocity;
			    float w = b.m_angularVelocity;

			    // Store positions for continuous collision.
			    b.m_sweep.c0 = b.m_sweep.c;
			    b.m_sweep.a0 = b.m_sweep.a;

			    if (b.m_type == BodyType._dynamicBody)
			    {
			        // Integrate velocities.
			        v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
			        w += h * b.m_invI * b.m_torque;

			        // Apply damping.
			        // ODE: dv/dt + c * v = 0
			        // Solution: v(t) = v0 * exp(-c * t)
			        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			        // v2 = exp(-c * dt) * v1
			        // Taylor expansion:
			        // v2 = (1.0f - c * dt) * v1
			        v *= Utilities.Clamp(1.0f - h * b.m_linearDamping, 0.0f, 1.0f);
					w *= Utilities.Clamp(1.0f - h * b.m_angularDamping, 0.0f, 1.0f);
			    }

				Position pos = new Position();
				pos.c = c;
				pos.a = a;
				m_positions.Add(pos);

				Velocity vel = new Velocity();
				vel.v = v;
				vel.w = w;
				m_velocities.Add(vel);
			}

			timer.Reset();

			// Solver data
			SolverData solverData;
			solverData.step = step;
			solverData.positions = m_positions;
			solverData.velocities = m_velocities;

			// Initialize velocity constraints.
			ContactSolverDef contactSolverDef;
			contactSolverDef.step = step;
			contactSolverDef.contacts = m_contacts;
			contactSolverDef.positions = m_positions;
			contactSolverDef.velocities = m_velocities;

			ContactSolver contactSolver = new ContactSolver(contactSolverDef);
			contactSolver.InitializeVelocityConstraints();

			if (step.warmStarting)
			{
			    contactSolver.WarmStart();
			}
	
			for (int i = 0; i < m_joints.Count(); ++i)
			{
			    m_joints[i].InitVelocityConstraints(solverData);
			}

			profile.solveInit = timer.GetMilliseconds();

			// Solve velocity constraints
			timer.Reset();
			for (int i = 0; i < step.velocityIterations; ++i)
			{
			    for (int j = 0; j < m_joints.Count(); ++j)
			    {
			        m_joints[j].SolveVelocityConstraints(solverData);
			    }

			    contactSolver.SolveVelocityConstraints();
			}

			// Store impulses for warm starting
			contactSolver.StoreImpulses();
			profile.solveVelocity = timer.GetMilliseconds();

			// Integrate positions
			for (int i = 0; i < m_bodies.Count(); ++i)
			{
			    Vec2 c = m_positions[i].c;
			    float a = m_positions[i].a;
			    Vec2 v = m_velocities[i].v;
			    float w = m_velocities[i].w;

			    // Check for large velocities
			    Vec2 translation = h * v;
			    if (Utilities.Dot(translation, translation) > Settings._maxTranslationSquared)
			    {
					float ratio = Settings._maxTranslation / translation.Length();
			        v *= ratio;
			    }

			    float rotation = h * w;
				if (rotation * rotation > Settings._maxRotationSquared)
			    {
					float ratio = Settings._maxRotation / Math.Abs(rotation);
			        w *= ratio;
			    }

			    // Integrate
			    c += h * v;
			    a += h * w;

				m_positions[i].c = c;
				m_positions[i].a = a;
				m_velocities[i].v = v;
				m_velocities[i].w = w;
			}

			// Solve position constraints
			timer.Reset();
			bool positionSolved = false;
			for (int i = 0; i < step.positionIterations; ++i)
			{
			    bool contactsOkay = contactSolver.SolvePositionConstraints();

			    bool jointsOkay = true;
			    for (int j = 0; j < m_joints.Count; ++j)
			    {
			        bool jointOkay = m_joints[j].SolvePositionConstraints(solverData);
			        jointsOkay = jointsOkay && jointOkay;
			    }

			    if (contactsOkay && jointsOkay)
			    {
			        // Exit early if the position errors are small.
			        positionSolved = true;
			        break;
			    }
			}

			// Copy state buffers back to the bodies
			for (int i = 0; i < m_bodies.Count(); ++i)
			{
			    Body body = m_bodies[i];
			    body.m_sweep.c = m_positions[i].c;
			    body.m_sweep.a = m_positions[i].a;
			    body.m_linearVelocity = m_velocities[i].v;
			    body.m_angularVelocity = m_velocities[i].w;
			    body.SynchronizeTransform();
			}

			profile.solvePosition = timer.GetMilliseconds();

			Report(contactSolver.m_velocityConstraints);

			if (allowSleep)
			{
			    float minSleepTime = Single.MaxValue;

				const float linTolSqr = Settings._linearSleepTolerance * Settings._linearSleepTolerance;
				const float angTolSqr = Settings._angularSleepTolerance * Settings._angularSleepTolerance;

			    for (int i = 0; i < m_bodies.Count(); ++i)
			    {
			        Body b = m_bodies[i];
			        if (b.GetBodyType() == BodyType._staticBody)
			        {
			            continue;
			        }

			        if ((b.m_flags & Body.BodyFlags.e_autoSleepFlag) == 0 ||
			            b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
			            Utilities.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
			        {
			            b.m_sleepTime = 0.0f;
			            minSleepTime = 0.0f;
			        }
			        else
			        {
			            b.m_sleepTime += h;
			            minSleepTime = Math.Min(minSleepTime, b.m_sleepTime);
			        }
			    }

				if (minSleepTime >= Settings._timeToSleep && positionSolved)
			    {
			        for (int i = 0; i < m_bodies.Count(); ++i)
			        {
			            Body b = m_bodies[i];
			            b.SetAwake(false);
			        }
			    }
			}
		}

		public void SolveTOI(TimeStep subStep, int toiIndexA, int toiIndexB) {
			throw new NotImplementedException();
		//    Utilities.Assert(toiIndexA < m_bodies.Count());
		//    Utilities.Assert(toiIndexB < m_bodies.Count());

		//    // Initialize the body state.
		//    for (int i = 0; i < m_bodies.Count(); ++i)
		//    {
		//        Body* b = m_bodies[i];
		//        m_positions[i].c = b.m_sweep.c;
		//        m_positions[i].a = b.m_sweep.a;
		//        m_velocities[i].v = b.m_linearVelocity;
		//        m_velocities[i].w = b.m_angularVelocity;
		//    }

		//    ContactSolverDef contactSolverDef;
		//    contactSolverDef.contacts = m_contacts;
		//    contactSolverDef.count = m_contactCount;
		//    contactSolverDef.allocator = m_allocator;
		//    contactSolverDef.step = subStep;
		//    contactSolverDef.positions = m_positions;
		//    contactSolverDef.velocities = m_velocities;
		//    ContactSolver contactSolver(&contactSolverDef);

		//    // Solve position constraints.
		//    for (int i = 0; i < subStep.positionIterations; ++i)
		//    {
		//        bool contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
		//        if (contactsOkay)
		//        {
		//            break;
		//        }
		//    }

		//#if ZERO
		//    // Is the new position really safe?
		//    for (int i = 0; i < m_contactCount; ++i)
		//    {
		//        Contact* c = m_contacts[i];
		//        Fixture fA = c.GetFixtureA();
		//        Fixture fB = c.GetFixtureB();

		//        Body* bA = fA.GetBody();
		//        Body* bB = fB.GetBody();

		//        int indexA = c.GetChildIndexA();
		//        int indexB = c.GetChildIndexB();

		//        DistanceInput input;
		//        input.proxyA.Set(fA.GetShape(), indexA);
		//        input.proxyB.Set(fB.GetShape(), indexB);
		//        input.transformA = bA.GetTransform();
		//        input.transformB = bB.GetTransform();
		//        input.useRadii = false;

		//        DistanceOutput output;
		//        SimplexCache cache;
		//        cache.count = 0;
		//        Utilities.Distance(out output, cache, input);

		//        if (output.distance == 0 || cache.count == 3)
		//        {
		//            cache.count += 0;
		//        }
		//    }
		//#endif

		//    // Leap of faith to new safe state.
		//    m_bodies[toiIndexA].m_sweep.c0 = m_positions[toiIndexA].c;
		//    m_bodies[toiIndexA].m_sweep.a0 = m_positions[toiIndexA].a;
		//    m_bodies[toiIndexB].m_sweep.c0 = m_positions[toiIndexB].c;
		//    m_bodies[toiIndexB].m_sweep.a0 = m_positions[toiIndexB].a;

		//    // No warm starting is needed for TOI events because warm
		//    // starting impulses were applied in the discrete solver.
		//    contactSolver.InitializeVelocityConstraints();

		//    // Solve velocity constraints.
		//    for (int i = 0; i < subStep.velocityIterations; ++i)
		//    {
		//        contactSolver.SolveVelocityConstraints();
		//    }

		//    // Don't store the TOI contact forces for warm starting
		//    // because they can be quite large.

		//    float h = subStep.dt;

		//    // Integrate positions
		//    for (int i = 0; i < m_bodies.Count(); ++i)
		//    {
		//        Vec2 c = m_positions[i].c;
		//        float a = m_positions[i].a;
		//        Vec2 v = m_velocities[i].v;
		//        float w = m_velocities[i].w;

		//        // Check for large velocities
		//        Vec2 translation = h * v;
		//        if (Utilities.Dot(translation, translation) > _maxTranslationSquared)
		//        {
		//            float ratio = _maxTranslation / translation.Length();
		//            v *= ratio;
		//        }

		//        float rotation = h * w;
		//        if (rotation * rotation > _maxRotationSquared)
		//        {
		//            float ratio = _maxRotation / Math.Abs(rotation);
		//            w *= ratio;
		//        }

		//        // Integrate
		//        c += h * v;
		//        a += h * w;

		//        m_positions[i].c = c;
		//        m_positions[i].a = a;
		//        m_velocities[i].v = v;
		//        m_velocities[i].w = w;

		//        // Sync bodies
		//        Body* body = m_bodies[i];
		//        body.m_sweep.c = c;
		//        body.m_sweep.a = a;
		//        body.m_linearVelocity = v;
		//        body.m_angularVelocity = w;
		//        body.SynchronizeTransform();
		//    }

		//    Report(contactSolver.m_velocityConstraints);
		}

		public void Add(Body body)
		{
			body.m_islandIndex = m_bodies.Count();
			m_bodies.Add(body);
		}

		public void Add(Contact contact)
		{
			m_contacts.Add(contact);
		}

		public void Add(Joint joint)
		{
			m_joints.Add(joint);
		}

		public void Report(List<ContactVelocityConstraint> constraints) {
			if (m_listener == null) {
				return;
			}

			for (int i = 0; i < m_contacts.Count(); ++i) {
				Contact c = m_contacts[i];

				ContactVelocityConstraint vc = constraints[i];

				ContactImpulse impulse = new ContactImpulse();
				impulse.count = vc.points.Count();
				impulse.normalImpulses.Clear();
				impulse.tangentImpulses.Clear();
				for (int j = 0; j < vc.points.Count(); ++j) {
					impulse.normalImpulses.Add(vc.points[j].normalImpulse);
					impulse.tangentImpulses.Add(vc.points[j].tangentImpulse);
				}

				m_listener.PostSolve(c, impulse);
			}
		}

		public ContactListener m_listener; //pointer

		public List<Body> m_bodies; //pointer to a pointer
		public List<Contact> m_contacts; //pointer to a pointer
		public List<Joint> m_joints; //pointer to a pointer

		public List<Position> m_positions; //pointer
		public List<Velocity> m_velocities; //pointer
		
	}
}
