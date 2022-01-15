package edu.unc.cs.gamma.rvo.twod

import edu.unc.cs.gamma.rvo.Ref
import edu.unc.cs.gamma.rvo.Utils.RVO_EPSILON
import edu.unc.cs.gamma.rvo.Utils.RVO_MAX_LEAF_SIZE
import edu.unc.cs.gamma.rvo.Utils.det
import edu.unc.cs.gamma.rvo.Utils.leftOf
import edu.unc.cs.gamma.rvo.Utils.sq
import edu.unc.cs.gamma.rvo.Utils.zero2d
import org.joml.Vector2d
import kotlin.math.max
import kotlin.math.min

class KdTree(val sim: RVOSimulator) {

    private val agents = ArrayList<Agent>()
    private val nodes = ArrayList<AgentTreeNode>()

    private var obstacleTree: ObstacleTreeNode? = null

    private val deadNodes = ArrayList<ObstacleTreeNode>()

    fun buildAgentTree() {
        agents.clear()
        agents.addAll(sim.agents)
        if (agents.isNotEmpty()) {
            val targetSize = 2 * agents.size - 1
            nodes.ensureCapacity(targetSize)
            while (nodes.size < targetSize) {
                nodes.add(AgentTreeNode())
            }
            buildAgentTreeRecursive(0, agents.size, 0)
        }
    }

    private fun buildAgentTreeRecursive(begin: Int, end: Int, nodeIndex: Int) {

        var node = nodes[nodeIndex]
        node.begin = begin
        node.end = end

        val nodeAgent = agents[begin]
        node.minCoord.set(nodeAgent.position)
        node.maxCoord.set(nodeAgent.position)

        for (i in begin + 1 until end) {
            val agentI = agents[i]
            node.minCoord.min(agentI.position)
            node.maxCoord.max(agentI.position)
        }

        if (end - begin > RVO_MAX_LEAF_SIZE) {
            val delta = node.maxCoord.sub(node.minCoord, node.delta)
            val coord = delta.maxComponent()
            val splitValue = (node.minCoord[coord] + node.maxCoord[coord]) * 0.5
            var left = begin
            var right = end
            while (left < right) {
                while (left < right && agents[left].position[coord] < splitValue) {
                    left++
                }
                while (right > left && agents[right - 1].position[coord] >= splitValue) {
                    right--
                }
                if (left < right) {
                    swapAgents(left, right - 1)
                    left++
                    right--
                }
            }

            var leftSize = left - begin
            if (leftSize == 0) {
                leftSize++
                left++
                // right++
            }

            // might have changed, I think
            node = nodes[nodeIndex]
            node.left = nodeIndex + 1
            node.right = nodeIndex + 2 * leftSize
            buildAgentTreeRecursive(begin, left, node.left)
            buildAgentTreeRecursive(left, end, node.right)

        }
    }

    private fun swapAgents(i: Int, j: Int) {
        val tmp = agents[i]
        agents[i] = agents[j]
        agents[j] = tmp
    }

    fun buildObstacleTree() {
        destroyObstacleTree(obstacleTree)
        val o = sim.obstacles
        val obstacles = Array(o.size) { o[it] }
        obstacleTree = buildObstacleTreeRecursive(obstacles.toList())
        deadNodes.clear() // free remaining memory space, if possible
    }

    private fun destroyObstacleTree(node: ObstacleTreeNode?) {
        node ?: return
        deadNodes.add(node)
        destroyObstacleTree(node.left)
        destroyObstacleTree(node.right)
    }

    private fun buildObstacleTreeRecursive(obstacles: List<Obstacle>): ObstacleTreeNode? {
        if (obstacles.isEmpty()) return null
        // reuse old nodes
        val node = if (deadNodes.isEmpty()) ObstacleTreeNode() else deadNodes.removeAt(deadNodes.lastIndex)
        var optimalSplit = 0
        var minLeft = obstacles.size
        var minRight = obstacles.size
        for (i in obstacles.indices) {
            var leftSize = 0
            var rightSize = 0
            val i1 = obstacles[i]
            val i2 = i1.nextObstacle!!
            // compute optimal split node
            for (j in obstacles.indices) {
                if (i == j) continue
                val j1 = obstacles[j]
                val j2 = j1.nextObstacle!!
                val j1LeftOfI = leftOf(i1.point, i2.point, j1.point)
                val j2LeftOfI = leftOf(i1.point, i2.point, j2.point)
                when {
                    j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON -> leftSize++
                    j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON -> rightSize++
                    else -> {
                        leftSize++
                        rightSize++
                    }
                }
                if (Pair(max(leftSize, rightSize), min(leftSize, rightSize)) >= Pair(
                        max(minLeft, minRight),
                        min(minLeft, minRight)
                    )
                ) break
            }
            if (Pair(max(leftSize, rightSize), min(leftSize, rightSize)) < Pair(
                    max(minLeft, minRight),
                    min(minLeft, minRight)
                )
            ) {
                minLeft = leftSize
                minRight = rightSize
                optimalSplit = i
            }
        }

        // build split node
        val leftObstacles = ArrayList<Obstacle>(minLeft)
        val rightObstacles = ArrayList<Obstacle>(minRight)
        val i = optimalSplit
        val i1 = obstacles[i]
        val i2 = i1.nextObstacle!!

        for (j in obstacles.indices) {
            if (i == j) continue

            val j1 = obstacles[j]
            val j2 = j1.nextObstacle!!

            val j1loi = leftOf(i1.point, i2.point, j1.point)
            val j2loi = leftOf(i1.point, i2.point, j2.point)

            when {
                j1loi >= -RVO_EPSILON && j2loi >= -RVO_EPSILON -> {
                    leftObstacles.add(j1)
                }
                j1loi <= RVO_EPSILON && j2loi <= RVO_EPSILON -> {
                    rightObstacles.add(j1)
                }
                else -> {
                    // split obstacle j
                    val t = det(i2.point, i1.point, j1.point, i1.point) /
                            det(i2.point, i1.point, j1.point, j2.point)
                    val splitPoint = Vector2d(j1.point)
                        .lerp(j2.point, t)
                    val newObstacle = Obstacle(
                        true, j2, j1, splitPoint, j1.unitDir,
                        sim.obstacles.size
                    )
                    sim.obstacles.add(newObstacle)
                    j1.nextObstacle = newObstacle
                    j2.prevObstacle = newObstacle
                    if (j1loi > 0.0) {
                        leftObstacles.add(j1)
                        rightObstacles.add(newObstacle)
                    } else {
                        rightObstacles.add(j1)
                        leftObstacles.add(newObstacle)
                    }
                }
            }

        }

        node.obstacle = i1
        node.left = buildObstacleTreeRecursive(leftObstacles)
        node.right = buildObstacleTreeRecursive(rightObstacles)
        return node
    }

    fun computeAgentNeighbors(agent: Agent, rangeSq: Double) {
        queryAgentTreeRecursive(agent, Ref(rangeSq), 0)
    }

    private fun queryAgentTreeRecursive(
        agent: Agent,
        rangeSq: Ref<Double>,
        nodeIndex: Int,
        tmp: Vector2d = Vector2d()
    ) {
        val node = nodes[nodeIndex]
        if (node.end - node.begin <= RVO_MAX_LEAF_SIZE) {
            for (i in node.begin until node.end) {
                agent.insertNeighbor(agents[i], rangeSq)
            }
        } else {
            val left = nodes[node.left]
            val right = nodes[node.right]
            val pos = agent.position
            val distSqLeft = distance(left, pos, tmp)
            val distSqRight = distance(right, pos, tmp)
            if (distSqLeft < distSqRight) {
                if (distSqLeft < rangeSq.value) {
                    queryAgentTreeRecursive(agent, rangeSq, node.left)
                    if (distSqRight < rangeSq.value) {
                        queryAgentTreeRecursive(agent, rangeSq, node.right)
                    }
                }
            } else {
                if (distSqRight < rangeSq.value) {
                    queryAgentTreeRecursive(agent, rangeSq, node.right)
                    if (distSqLeft < rangeSq.value) {
                        queryAgentTreeRecursive(agent, rangeSq, node.left)
                    }
                }
            }
        }
    }

    fun computeObstacleNeighbors(agent: Agent, rangeSq: Double) {
        queryObstacleTreeRecursive(agent, rangeSq, obstacleTree)
    }

    private fun queryObstacleTreeRecursive(agent: Agent, rangeSq: Double, node: ObstacleTreeNode?) {
        if (node == null) return
        val o1 = node.obstacle!!
        val o2 = o1.nextObstacle!!
        val agentLeftOfLine = leftOf(o1.point, o2.point, agent.position)
        queryObstacleTreeRecursive(agent, rangeSq, if (agentLeftOfLine >= 0.0) node.left else node.right)
        val distSqLine = sq(agentLeftOfLine) / o2.point.distanceSquared(o1.point)
        if (distSqLine < rangeSq) {
            if (agentLeftOfLine < 0.0) {
                //  Try obstacle at this node only if agent is on right side of obstacle (and can see obstacle).
                agent.insertObstacleNeighbor(node.obstacle!!, rangeSq)
            }
            // try other side of line
            queryObstacleTreeRecursive(agent, rangeSq, if (agentLeftOfLine >= 0.0) node.right else node.left)
        }
    }

    fun queryVisibility(q1: Vector2d, q2: Vector2d, radius: Double): Boolean {
        return queryVisibilityRecursive(q1, q2, radius, obstacleTree)
    }

    private fun queryVisibilityRecursive(q1: Vector2d, q2: Vector2d, radius: Double, node: ObstacleTreeNode?): Boolean {
        if (node == null) return true
        val o1 = node.obstacle!!
        val o2 = o1.nextObstacle!!
        val q1loi = leftOf(o1.point, o2.point, q1)
        val q2loi = leftOf(o1.point, o2.point, q2)
        val invLengthI = 1.0 / o2.point.distanceSquared(o1.point)
        val r2 = sq(radius)
        val b1 = sq(q1loi) * invLengthI >= r2
        val b2 = sq(q2loi) * invLengthI >= r2
        return when {
            q1loi >= 0.0 && q2loi >= 0.0 -> {
                queryVisibilityRecursive(q1, q2, radius, node.left) &&
                        ((b1 && b2) || queryVisibilityRecursive(q1, q2, radius, node.right))
            }
            q1loi <= 0.0 && q2loi <= 0.0 -> {
                queryVisibilityRecursive(q1, q2, radius, node.right) &&
                        ((b1 && b2) || queryVisibilityRecursive(q1, q2, radius, node.left))
            }
            q1loi >= 0.0 && q2loi <= 0.0 -> {
                queryVisibilityRecursive(q1, q2, radius, node.left) &&
                        queryVisibilityRecursive(q1, q2, radius, node.right)
            }
            else -> {
                val p1loq = leftOf(q1, q2, o1.point)
                val p2loq = leftOf(q1, q2, o2.point)
                val invLengthQ = 1.0 / q2.distanceSquared(q1)
                p1loq * p2loq >= 0.0 &&
                        sq(p1loq) * invLengthQ > r2 &&
                        sq(p2loq) * invLengthQ > r2 &&
                        queryVisibilityRecursive(q1, q2, radius, node.left) &&
                        queryVisibilityRecursive(q1, q2, radius, node.right)
            }
        }
    }

    private fun distance(left: AgentTreeNode, pos: Vector2d, tmp: Vector2d): Double {
        return tmp.set(left.minCoord).sub(pos).max(zero2d).lengthSquared() +
                tmp.set(pos).sub(left.maxCoord).max(zero2d).lengthSquared()
    }

    operator fun <A : Comparable<A>, B : Comparable<B>> Pair<A, B>.compareTo(other: Pair<A, B>): Int {
        val l0 = first.compareTo(other.first)
        return if (l0 == 0) second.compareTo(other.second) else l0
    }

}