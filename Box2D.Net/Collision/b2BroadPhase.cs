using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
	/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
	/// It is up to the client to consume the new pairs and to track subsequent overlap.
	class b2BroadPhase
	{
		public static bool b2PairLessThan(b2Pair pair1, b2Pair pair2)
		{
			if (pair1.proxyIdA < pair2.proxyIdA)
			{
				return true;
			}

			if (pair1.proxyIdA == pair2.proxyIdA)
			{
				return pair1.proxyIdB < pair2.proxyIdB;
			}

			return false;
		}

		public enum NullProxy
		{
			e_nullProxy = -1
		}

		public b2BroadPhase(){
			m_proxyCount = 0;

			m_pairCapacity = 16;
			m_pairCount = 0;
			m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));

			m_moveCapacity = 16;
			m_moveCount = 0;
			m_moveBuffer = (float*)b2Alloc(m_moveCapacity * sizeof(float));
		}

		public ~b2BroadPhase(){
			throw new NotImplementedException();
			//b2Free(m_moveBuffer);
			//b2Free(m_pairBuffer);
		}

		/// Create a proxy with an initial AABB. Pairs are not reported until
		/// UpdatePairs is called.
		public int CreateProxy(b2AABB aabb, object userData){
			throw new NotImplementedException();
			//float proxyId = m_tree.CreateProxy(aabb, userData);
			//++m_proxyCount;
			//BufferMove(proxyId);
			//return proxyId;
		}

		/// Destroy a proxy. It is up to the client to remove any pairs.
		public void DestroyProxy(int proxyId){
			throw new NotImplementedException();
			//UnBufferMove(proxyId);
			//--m_proxyCount;
			//m_tree.DestroyProxy(proxyId);
		}

		/// Call MoveProxy as many times as you like, then when you are done
		/// call UpdatePairs to finalized the proxy pairs (for your time step).
		public void MoveProxy(int proxyId, b2AABB aabb, b2Vec2 displacement){
			throw new NotImplementedException();
			//bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
			//if (buffer)
			//{
			//    BufferMove(proxyId);
			//}
		}

		/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
		public void TouchProxy(int proxyId){
			BufferMove(proxyId);
		}

		/// Get the fat AABB for a proxy.
		public b2AABB GetFatAABB(int proxyId){
			throw new NotImplementedException();
			//return m_tree.GetFatAABB(proxyId);
		}


		/// Get user data from a proxy. Returns null if the id is invalid.
		public object GetUserData(int proxyId){
			return m_tree.GetUserData(proxyId);
		}

		/// Test overlap of fat AABBs.
		public bool TestOverlap(int proxyIdA, int proxyIdB){
			b2AABB aabbA = m_tree.GetFatAABB(proxyIdA);
			b2AABB aabbB = m_tree.GetFatAABB(proxyIdB);
			throw new NotImplementedException();
			//return b2TestOverlap(aabbA, aabbB);
		}

		/// Get the number of proxies.
		public int GetProxyCount(){
			return m_proxyCount;
		}

		/// Update the pairs. This results in pair callbacks. This can only add pairs.
		public void UpdatePairs<T>(T callback){
			throw new NotImplementedException();
			//// Reset pair buffer
			//m_pairCount = 0;

			//// Perform tree queries for all moving proxies.
			//for (int i = 0; i < m_moveCount; ++i)
			//{
			//    m_queryProxyId = m_moveBuffer[i];
			//    if (m_queryProxyId == e_nullProxy)
			//    {
			//        continue;
			//    }

			//    // We have to query the tree with the fat AABB so that
			//    // we don't fail to create a pair that may touch later.
			//    const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

			//    // Query tree, create pairs and add them pair buffer.
			//    m_tree.Query(this, fatAABB);
			//}

			//// Reset move buffer
			//m_moveCount = 0;

			//// Sort the pair buffer to expose duplicates.
			//std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, b2PairLessThan);

			//// Send the pairs back to the client.
			//int i = 0;
			//while (i < m_pairCount)
			//{
			//    b2Pair* primaryPair = m_pairBuffer + i;
			//    void* userDataA = m_tree.GetUserData(primaryPair.proxyIdA);
			//    void* userDataB = m_tree.GetUserData(primaryPair.proxyIdB);

			//    callback.AddPair(userDataA, userDataB);
			//    ++i;

			//    // Skip any duplicate pairs.
			//    while (i < m_pairCount)
			//    {
			//        b2Pair* pair = m_pairBuffer + i;
			//        if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
			//        {
			//            break;
			//        }
			//        ++i;
			//    }
			//}

			//// Try to keep the tree balanced.
			////m_tree.Rebalance(4);
		}

		/// Query an AABB for overlapping proxies. The callback class
		/// is called for each proxy that overlaps the supplied AABB.
		public void Query<T>(T callback, b2AABB aabb){
			m_tree.Query(callback, aabb);
		}

		/// Ray-cast against the proxies in the tree. This relies on the callback
		/// to perform a exact ray-cast in the case were the proxy contains a shape.
		/// The callback also performs the any collision filtering. This has performance
		/// roughly equal to k * log(n), where k is the number of collisions and n is the
		/// number of proxies in the tree.
		/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
		/// @param callback a callback class that is called for each proxy that is hit by the ray.
		void RayCast<T>(T callback, b2RayCastInput input){
			m_tree.RayCast(callback, input);
		}

		/// Get the height of the embedded tree.
		public int GetTreeHeight(){
			return m_tree.GetHeight();
		}

		/// Get the balance of the embedded tree.
		public int GetTreeBalance(){
			return m_tree.GetMaxBalance();
		}

		/// Get the quality metric of the embedded tree.
		public float GetTreeQuality(){
			return m_tree.GetAreaRatio();
		}

		/// Shift the world origin. Useful for large worlds.
		/// The shift formula is: position -= newOrigin
		/// @param newOrigin the new origin with respect to the old origin
		public void ShiftOrigin(b2Vec2 newOrigin){
			m_tree.ShiftOrigin(newOrigin);
		}

		private void BufferMove(int proxyId){
			throw new NotImplementedException();
			//if (m_moveCount == m_moveCapacity)
			//{
			//    float* oldBuffer = m_moveBuffer;
			//    m_moveCapacity *= 2;
			//    m_moveBuffer = (float*)b2Alloc(m_moveCapacity * sizeof(float));
			//    memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(float));
			//    b2Free(oldBuffer);
			//}

			//m_moveBuffer[m_moveCount] = proxyId;
			//++m_moveCount;
		}
		private void UnBufferMove(int proxyId){
			throw new NotImplementedException();
			//for (float i = 0; i < m_moveCount; ++i)
			//{
			//    if (m_moveBuffer[i] == proxyId)
			//    {
			//        m_moveBuffer[i] = e_nullProxy;
			//    }
			//}
		}

		private bool QueryCallback(int proxyId){
			throw new NotImplementedException();
			//// A proxy cannot form a pair with itself.
			//if (proxyId == m_queryProxyId)
			//{
			//    return true;
			//}

			//// Grow the pair buffer as needed.
			//if (m_pairCount == m_pairCapacity)
			//{
			//    b2Pair* oldBuffer = m_pairBuffer;
			//    m_pairCapacity *= 2;
			//    m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));
			//    memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(b2Pair));
			//    b2Free(oldBuffer);
			//}

			//m_pairBuffer[m_pairCount].proxyIdA = Math.Min(proxyId, m_queryProxyId);
			//m_pairBuffer[m_pairCount].proxyIdB = b2Max(proxyId, m_queryProxyId);
			//++m_pairCount;

			//return true;
		}


		private b2DynamicTree m_tree;

		private int m_proxyCount;

		private int* m_moveBuffer;
		private int m_moveCapacity;
		private int m_moveCount;

		private b2Pair* m_pairBuffer;
		private int m_pairCapacity;
		private int m_pairCount;

		private int m_queryProxyId;
	};
}
