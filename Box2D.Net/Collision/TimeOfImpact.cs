using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public partial class Utilities {
		public static int _toiCalls, _toiIters, _toiMaxIters;
		public static float _toiTime, _toiMaxTime;
		public static int _toiRootIters, _toiMaxRootIters;

		/// Compute the upper bound on time before two shapes penetrate. Time is represented as
		/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
		/// non-tunneling collision. If you change the time interval, you should call this function
		/// again.
		/// Note: use Distance to compute the contact point and normal at the time of impact.
		// CCD via the local separating axis method. This seeks progression
		// by computing the largest time at which separation is maintained.
		public static void TimeOfImpact(out TOIOutput output, TOIInput input){
		    Timer timer = new Timer();

		    ++_toiCalls;

		    output.state = TOIOutput.State.e_unknown;
		    output.t = input.tMax;

		    DistanceProxy proxyA = input.proxyA;
		    DistanceProxy proxyB = input.proxyB;

		    Sweep sweepA = input.sweepA;
		    Sweep sweepB = input.sweepB;

		    // Large rotations can make the root finder fail, so we normalize the
		    // sweep angles.
		    sweepA.Normalize();
		    sweepB.Normalize();

		    float tMax = input.tMax;

		    float totalRadius = proxyA.m_radius + proxyB.m_radius;
			float target = Math.Max(Settings._linearSlop, totalRadius - 3.0f * Settings._linearSlop);
			float tolerance = 0.25f * Settings._linearSlop;
		    Utilities.Assert(target > tolerance);

		    float t1 = 0.0f;
		    const int k_maxIterations = 20;	// TODO_ERIN Settings
		    int iter = 0;

		    // Prepare input for distance query.
		    SimplexCache cache = new SimplexCache();
		    cache.count = 0;
		    DistanceInput distanceInput;
		    distanceInput.proxyA = input.proxyA;
		    distanceInput.proxyB = input.proxyB;
		    distanceInput.useRadii = false;

		    // The outer loop progressively attempts to compute new separating axes.
		    // This loop terminates when an axis is repeated (no progress is made).
		    for(;;)
		    {
		        Transform xfA, xfB;
		        sweepA.GetTransform(out xfA, t1);
		        sweepB.GetTransform(out xfB, t1);

		        // Get the distance between shapes. We can also use the results
		        // to get a separating axis.
		        distanceInput.transformA = xfA;
		        distanceInput.transformB = xfB;
		        DistanceOutput distanceOutput;
				Utilities.Distance(out distanceOutput, cache, distanceInput);

		        // If the shapes are overlapped, we give up on continuous collision.
		        if (distanceOutput.distance <= 0.0f)
		        {
		            // Failure!
		            output.state = TOIOutput.State.e_overlapped;
		            output.t = 0.0f;
		            break;
		        }

		        if (distanceOutput.distance < target + tolerance)
		        {
		            // Victory!
		            output.state = TOIOutput.State.e_touching;
		            output.t = t1;
		            break;
		        }

		        // Initialize the separating axis.
				throw new NotImplementedException();
		//        SeparationFunction fcn;
		//        fcn.Initialize(&cache, proxyA, sweepA, proxyB, sweepB, t1);
		//#if ZERO
		//        // Dump the curve seen by the root finder
		//        {
		//            const int N = 100;
		//            float dx = 1.0f / N;
		//            float xs[N+1];
		//            float fs[N+1];

		//            float x = 0.0f;

		//            for (int i = 0; i <= N; ++i)
		//            {
		//                sweepA.GetTransform(out xfA, x);
		//                sweepB.GetTransform(out xfB, x);
		//                float f = fcn.Evaluate(xfA, xfB) - target;

		//                printf("%g %g\n", x, f);

		//                xs[i] = x;
		//                fs[i] = f;

		//                x += dx;
		//            }
		//        }
		//#endif

		//        // Compute the TOI on the separating axis. We do this by successively
		//        // resolving the deepest point. This loop is bounded by the number of vertices.
		//        bool done = false;
		//        float t2 = tMax;
		//        int pushBackIter = 0;
		//        for (;;)
		//        {
		//            // Find the deepest point at t2. Store the witness point indices.
		//            int indexA, indexB;
		//            float s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);

		//            // Is the final configuration separated?
		//            if (s2 > target + tolerance)
		//            {
		//                // Victory!
		//                output.state = TOIOutput.State.e_separated;
		//                output.t = tMax;
		//                done = true;
		//                break;
		//            }

		//            // Has the separation reached tolerance?
		//            if (s2 > target - tolerance)
		//            {
		//                // Advance the sweeps
		//                t1 = t2;
		//                break;
		//            }

		//            // Compute the initial separation of the witness points.
		//            float s1 = fcn.Evaluate(indexA, indexB, t1);

		//            // Check for initial overlap. This might happen if the root finder
		//            // runs out of iterations.
		//            if (s1 < target - tolerance)
		//            {
		//                output.state = TOIOutput.State.e_failed;
		//                output.t = t1;
		//                done = true;
		//                break;
		//            }

		//            // Check for touching
		//            if (s1 <= target + tolerance)
		//            {
		//                // Victory! t1 should hold the TOI (could be 0.0).
		//                output.state = TOIOutput.State.e_touching;
		//                output.t = t1;
		//                done = true;
		//                break;
		//            }

		//            // Compute 1D root of: f(x) - target = 0
		//            int rootIterCount = 0;
		//            float a1 = t1, a2 = t2;
		//            for (;;)
		//            {
		//                // Use a mix of the secant rule and bisection.
		//                float t;
		//                if (rootIterCount & 1)
		//                {
		//                    // Secant rule to improve convergence.
		//                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
		//                }
		//                else
		//                {
		//                    // Bisection to guarantee progress.
		//                    t = 0.5f * (a1 + a2);
		//                }

		//                ++rootIterCount;
		//                ++_toiRootIters;

		//                float s = fcn.Evaluate(indexA, indexB, t);

		//                if (Math.Abs(s - target) < tolerance)
		//                {
		//                    // t2 holds a tentative value for t1
		//                    t2 = t;
		//                    break;
		//                }

		//                // Ensure we continue to bracket the root.
		//                if (s > target)
		//                {
		//                    a1 = t;
		//                    s1 = s;
		//                }
		//                else
		//                {
		//                    a2 = t;
		//                    s2 = s;
		//                }
				
		//                if (rootIterCount == 50)
		//                {
		//                    break;
		//                }
		//            }

		//            _toiMaxRootIters = Math.Max(_toiMaxRootIters, rootIterCount);

		//            ++pushBackIter;

		//            if (pushBackIter == Settings._maxPolygonVertices)
		//            {
		//                break;
		//            }
		//        }

		//        ++iter;
		//        ++_toiIters;

		//        if (done)
		//        {
		//            break;
		//        }

		//        if (iter == k_maxIterations)
		//        {
		//            // Root finder got stuck. Semi-victory.
		//            output.state = TOIOutput.State.e_failed;
		//            output.t = t1;
		//            break;
		//        }
		    }

		    _toiMaxIters = Math.Max(_toiMaxIters, iter);

		    float time = timer.GetMilliseconds();
		    _toiMaxTime = Math.Max(_toiMaxTime, time);
		    _toiTime += time;
		}
	}
}
