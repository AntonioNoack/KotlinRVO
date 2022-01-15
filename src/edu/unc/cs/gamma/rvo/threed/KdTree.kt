package edu.unc.cs.gamma.rvo.threed

import edu.unc.cs.gamma.rvo.Ref
import edu.unc.cs.gamma.rvo.Utils.RVO_MAX_LEAF_SIZE
import edu.unc.cs.gamma.rvo.Utils.zero3d
import org.joml.Vector3d

class KdTree(val sim: RVOSimulator) {

    private val agents = ArrayList<Agent>()
    private val nodes = ArrayList<AgentTreeNode>()

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

    fun buildAgentTreeRecursive(begin: Int, end: Int, nodeIndex: Int) {

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
                    swap(left, right - 1)
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

    private fun swap(i: Int, j: Int) {
        val tmp = agents[i]
        agents[i] = agents[j]
        agents[j] = tmp
    }

    fun computeAgentNeighbors(agent: Agent, rangeSq: Double) {
        queryAgentTreeRecursive(agent, Ref(rangeSq), 0)
    }

    fun queryAgentTreeRecursive(agent: Agent, rangeSq: Ref<Double>, nodeIndex: Int, tmp: Vector3d = Vector3d()) {
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

    fun distance(left: AgentTreeNode, pos: Vector3d, tmp: Vector3d): Double {
        return tmp.set(left.minCoord).sub(pos).max(zero3d).lengthSquared() +
                tmp.set(pos).sub(left.maxCoord).max(zero3d).lengthSquared()
    }

}