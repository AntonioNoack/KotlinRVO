package edu.unc.cs.gamma.rvo.test

import edu.unc.cs.gamma.rvo.Utils.sq
import edu.unc.cs.gamma.rvo.threed.RVOSimulator
import org.joml.Vector3d

fun main() {

    runTest("3d", false, { timeStep, radius, speed ->
        RVOSimulator(timeStep, radius * 10.0, 16, 20.0, radius, speed)
    }, { sim, x, y ->
        sim.addAgent(Vector3d(x, y, 0.0))
    }, { agent, x, y ->
        agent.position.distanceSquared(x, y, 0.0)
    }, { agent, targetX, targetY ->
        agent.targetVelocity
            .set(targetX, targetY, 0.0)
            .sub(agent.position)
        if (agent.targetVelocity.lengthSquared() > sq(agent.maxSpeed)) {
            agent.targetVelocity.normalize(agent.maxSpeed)
        }
    }, { it.step() }, { it.position.x }, { it.position.y })

}