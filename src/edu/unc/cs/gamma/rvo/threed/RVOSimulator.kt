package edu.unc.cs.gamma.rvo.threed

import org.joml.Vector3d

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
    var radius: Double,
    var maxSpeed: Double,
    var velocity: Vector3d = Vector3d()
) {

    val kdTree = KdTree(this)
    val agents = ArrayList<Agent>()
    var globalTime = 0.0

    fun addAgent(position: Vector3d): Agent {
        return addAgent(position, neighborDist, maxNeighbors, timeHorizon, radius, maxSpeed, Vector3d(velocity))
    }

    fun addAgent(
        position: Vector3d,
        neighborDist: Double,
        maxNeighbors: Int,
        timeHorizon: Double,
        radius: Double,
        maxSpeed: Double,
        velocity: Vector3d
    ): Agent {
        val agent = Agent(
            this, position, maxNeighbors, maxSpeed,
            neighborDist, radius, timeHorizon, velocity,
            agents.size
        )
        agents.add(agent)
        return agent
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