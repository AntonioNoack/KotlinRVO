package edu.unc.cs.gamma.rvo.twod

import edu.unc.cs.gamma.rvo.Utils.leftOf
import org.joml.Vector2d

/**
 * Constructs a simulator instance and sets the default properties for any new agent that is added.
 * @param timeStep      The time step of the simulation. Must be positive.
 * @param neighborDist  The default maximum distance (center point to center point) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
 * @param maxNeighbors  The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
 * @param timeHorizon   The default minimum amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
 * @param radius        The default radius of a new agent. Must be non-negative.
 * @param maxSpeed      The default maximum speed of a new agent. Must be non-negative.
 * @param velocity      The default initial three-dimensional linear velocity of a new agent (optional).
 */
class RVOSimulator(
    var timeStep: Double,
    var neighborDist: Double,
    var maxNeighbors: Int,
    var timeHorizon: Double,
    var timeHorizonObstacles: Double,
    var radius: Double,
    var maxSpeed: Double,
    var velocity: Vector2d = Vector2d()
) {

    val kdTree = KdTree(this)
    val agents = ArrayList<Agent>()
    val obstacles = ArrayList<Obstacle>()
    var globalTime = 0.0

    fun addAgent(position: Vector2d): Agent {
        return addAgent(
            position,
            neighborDist,
            maxNeighbors,
            timeHorizon,
            timeHorizonObstacles,
            radius,
            maxSpeed,
            Vector2d(velocity)
        )
    }

    fun addAgent(
        position: Vector2d,
        neighborDist: Double,
        maxNeighbors: Int,
        timeHorizon: Double,
        timeHorizonObstacles: Double,
        radius: Double,
        maxSpeed: Double,
        velocity: Vector2d
    ): Agent {
        val agent = Agent(
            this, position, maxNeighbors, maxSpeed,
            neighborDist, radius, timeHorizon, timeHorizonObstacles,
            velocity, agents.size
        )
        agents.add(agent)
        return agent
    }

    fun addObstacle(vertices: List<Vector2d>): Int {
        if (vertices.size < 2) throw IllegalArgumentException("Shape must contain at least two vertices")
        val size = vertices.size
        val indexOfFirst = obstacles.size
        for (i in vertices.indices) {
            val point = vertices[i]
            val obstacle = Obstacle(
                size == 2,
                null, null,
                point, Vector2d(),
                obstacles.size
            )
            if (i > 0) {
                val prev = obstacles.last()
                prev.nextObstacle = obstacle
                obstacle.prevObstacle = prev
            }
            if (i == size - 1) {
                val first = obstacles[indexOfFirst]
                obstacle.nextObstacle = first
                first.prevObstacle = obstacle
            }

            obstacle.unitDir.set(vertices[if (i == size - 1) 0 else i + 1]).sub(point).normalize()

            if (size > 2) {
                obstacle.isConvex = leftOf(
                    vertices[if (i == 0) size - 1 else i - 1],
                    point,
                    vertices[if (i == size - 1) 0 else i + 1]
                ) >= 0.0
            }

            obstacles.add(obstacle)
        }
        return indexOfFirst
    }

    fun processObstacles() {
        kdTree.buildObstacleTree()
    }

    fun step() {
        kdTree.buildAgentTree()
        for (agent in agents) {
            agent.computeNeighbors()
            agent.computeNewVelocity()
        }
        for (agent in agents) {
            agent.update()
        }
        globalTime += timeStep
    }

}