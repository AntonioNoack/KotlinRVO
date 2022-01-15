package edu.unc.cs.gamma.rvo.test

import edu.unc.cs.gamma.rvo.Utils.sq
import edu.unc.cs.gamma.rvo.twod.RVOSimulator
import org.joml.Vector2d


fun main() {

    runTest("2d", false, 6.0, { timeStep, radius, speed ->
        RVOSimulator(
            timeStep, radius * 6.0, 6,
            5.0, 5.0, radius, speed
        )
    }, { sim, x, y ->
        sim.addAgent(Vector2d(x, y))
    }, { agent, x, y ->
        agent.position.distanceSquared(x, y)
    }, { agent, targetX, targetY ->
        agent.targetVelocity
            .set(targetX, targetY)
            .sub(agent.position)
        if (agent.targetVelocity.lengthSquared() > sq(agent.maxSpeed)) {
            agent.targetVelocity.normalize(agent.maxSpeed)
        }
    }, { it.step() }, { it.position.x }, { it.position.y })

}