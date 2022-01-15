package edu.unc.cs.gamma.rvo.twod

import edu.unc.cs.gamma.rvo.DistancedPair
import edu.unc.cs.gamma.rvo.Line
import edu.unc.cs.gamma.rvo.Ref
import edu.unc.cs.gamma.rvo.Utils.RVO_EPSILON
import edu.unc.cs.gamma.rvo.Utils.clamp
import edu.unc.cs.gamma.rvo.Utils.det
import edu.unc.cs.gamma.rvo.Utils.sq
import edu.unc.cs.gamma.rvo.Utils.zero2d
import org.joml.Vector2d
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

class Agent(
    val sim: RVOSimulator,
    val position: Vector2d,
    val maxNeighbors: Int,
    val maxSpeed: Double,
    val neighborDist: Double,
    val radius: Double,
    val timeHorizon: Double,
    val timeHorizonObstacles: Double,
    val velocity: Vector2d,
    val id: Int
) {

    val targetVelocity = Vector2d()
    val newVelocity = Vector2d()

    val neighbors = ArrayList<DistancedPair<Agent>>()
    val obstacleNeighbors = ArrayList<DistancedPair<Obstacle>>()

    private val orcaLines = ArrayList<Line<Vector2d>>()

    fun update() {
        if (newVelocity.lengthSquared() > sq(maxSpeed)) {
            if (newVelocity.lengthSquared() > sq(maxSpeed) * 1.01) {
                println("warn: $id was too last by ${newVelocity.length() / maxSpeed}x")
            }
            newVelocity.normalize(maxSpeed)
        }
        if (newVelocity.x.isNaN()) newVelocity.set(0.0)
        velocity.set(newVelocity)
        position.add(
            sim.timeStep * velocity.x,
            sim.timeStep * velocity.y
        )
    }

    fun computeNeighbors() {
        obstacleNeighbors.clear()
        sim.kdTree.computeObstacleNeighbors(this, sq(timeHorizonObstacles * maxSpeed + radius))
        neighbors.clear()
        if (maxNeighbors > 0) {
            sim.kdTree.computeAgentNeighbors(this, neighborDist * neighborDist)
        }
    }

    fun insertNeighbor(agent: Agent, rangeSq: Ref<Double>) {
        if (this !== agent) {
            val distDq = position.distanceSquared(agent.position)
            if (distDq < rangeSq.value) {
                if (neighbors.size < maxNeighbors) {
                    neighbors.add(DistancedPair(distDq, agent))
                }
                var i = neighbors.lastIndex
                while (i != 0 && distDq < neighbors[i].distSq) {
                    neighbors[i].set(neighbors[i - 1]) // ??...
                    i--
                }
                neighbors[i].distSq = distDq
                neighbors[i].instance = agent
                if (neighbors.size == maxNeighbors) {
                    rangeSq.value = neighbors.last().distSq
                }
            }
        }
    }

    fun insertObstacleNeighbor(obstacle: Obstacle, rangeSq: Double) {
        val next = obstacle.nextObstacle!!
        val distSq = distSqPointLineSegment(obstacle.point, next.point, position)
        if (distSq < rangeSq) {
            obstacleNeighbors.add(DistancedPair(distSq, obstacle))
            var i = obstacleNeighbors.size
            while (i != 0 && distSq < obstacleNeighbors[i - 1].distSq) {
                obstacleNeighbors[i].set(obstacleNeighbors[i - 1])
                i--
            }
            obstacleNeighbors[i].set(distSq, obstacle)
        }
    }

    private val rp1 = Vector2d()
    private val rp2 = Vector2d()

    fun computeNewVelocity() {
        orcaLines.clear()
        val invTimeHorizonObst = 1.0 / timeHorizonObstacles

        // Create obstacle ORCA lines
        obstacles@ for (i in obstacleNeighbors.indices) {
            val o1 = obstacleNeighbors[i].instance
            val o2 = o1.nextObstacle!!
            rp1.set(o1.point).sub(position)
            rp2.set(o2.point).sub(position)
            // Check if velocity obstacle of obstacle is already taken care of by
            // previously constructed obstacle ORCA lines.
            for (j in orcaLines.indices) {
                val orca = orcaLines[i]
                val dr = invTimeHorizonObst * radius
                if (det(invTimeHorizonObst, rp1, orca.point, orca.direction, zero2d)
                    - dr >= -RVO_EPSILON &&
                    det(invTimeHorizonObst, rp2, orca.point, orca.direction, zero2d) - dr >= -RVO_EPSILON
                ) {
                    continue@obstacles
                }
            }

            val distSq1 = rp1.lengthSquared()
            val distSq2 = rp2.lengthSquared()
            val radiusSq = sq(radius)
            val obstacleVec = Vector2d(o2.point).sub(o1.point)
            val s = -rp1.dot(obstacleVec) / obstacleVec.lengthSquared()
            val scaled = Vector2d(obstacleVec).mul(s)
            val distSqLine = scaled.distanceSquared(rp1)

            when {
                s < 0.0 && distSq1 <= radiusSq -> {
                    // Collision with left vertex. Ignore if non-convex.
                    if (o1.isConvex) {
                        val line = Line(Vector2d(), Vector2d())
                        line.point.set(0.0)
                        line.direction.set(-rp1.y, rp1.x).normalize()
                        orcaLines.add(line)
                    }
                    continue@obstacles
                }
                s > 1.0 && distSq2 <= radiusSq -> {
                    // Collision with right vertex. Ignore if non-convex
                    // or if it will be taken care of by neighoring obstace
                    if (o2.isConvex && det(rp2, o2.unitDir) >= 0.0) {
                        val line = Line(Vector2d(), Vector2d())
                        line.point.set(0.0)
                        line.direction.set(-rp2.y, rp2.x).normalize()
                        orcaLines.add(line)
                    }
                    continue@obstacles
                }
                s >= 0.0 && s < 1.0 && distSqLine <= radiusSq -> {
                    val line = Line(Vector2d(), Vector2d())
                    line.point.set(0.0)
                    line.direction.set(o1.unitDir).mul(-1.0)
                    orcaLines.add(line)
                    continue@obstacles
                }
            }

            // No collision.
            // Compute legs. When obliquely viewed, both legs can come from a single
            // vertex. Legs extend cut-off line when nonconvex vertex.

            // long stuff coming...
            TODO()

        }

        val numObstLines = orcaLines.size
        val invTimeHorizon = 1.0 / timeHorizon

        val relPos = relPos
        val relVel = relVel
        for (i in neighbors.indices) {
            val other = neighbors[i].instance
            relPos.set(other.position).sub(position)
            relVel.set(velocity).sub(other.velocity)
            val distSq = relPos.lengthSquared()
            val combinedRadius = radius + other.radius
            val combinedRadiusSq = sq(combinedRadius)

            val line = Line(Vector2d(), Vector2d())
            val u = Vector2d()
            val w = Vector2d()

            if (distSq > combinedRadiusSq) {
                // no collision
                w.set(relPos).mul(-invTimeHorizon).add(relVel)
                val wLengthSq = w.lengthSquared()
                val dot1 = w.dot(relPos)
                if (dot1 < 0.0 && sq(dot1) > combinedRadiusSq * wLengthSq) {
                    // project on cut-off circle
                    val wLength = sqrt(wLengthSq)
                    w.div(wLength)
                    u.set(w).mul(combinedRadius * invTimeHorizon - wLength)
                } else {
                    // project on legs
                    val leg = sqrt(distSq - combinedRadiusSq)
                    if (det(relPos, w) > 0.0) {
                        // left leg
                        line.direction.set(
                            relPos.x * leg - relPos.y * combinedRadius,
                            relPos.x * combinedRadius + relPos.y * leg
                        ).div(distSq)
                    } else {
                        // right leg
                        line.direction.set(
                            relPos.x * leg + relPos.y * combinedRadius,
                            -relPos.x * combinedRadius + relPos.y * leg
                        ).div(distSq)
                    }
                    val dot2 = relPos.dot(line.direction)
                    u.set(line.direction)
                        .mul(dot2).sub(relVel)
                }
            } else {
                // collision, project on cut-off circle of time timeStep
                val invTimeStep = 1.0 / sim.timeStep
                w.set(relPos).mul(-invTimeStep).add(relVel)
                val wLength = w.length()
                w.div(wLength)
                line.direction.set(w.y, -w.x)
                u.set(w).mul(combinedRadius * invTimeStep - wLength)
            }

            line.point.set(u).mul(0.5).add(velocity)
            orcaLines.add(line)

        }

        val lineFail = linearProgram2(orcaLines, maxSpeed, targetVelocity, false, newVelocity)
        if (lineFail < orcaLines.size) {
            linearProgram3(orcaLines, numObstLines, lineFail, maxSpeed, newVelocity)
        }

    }

    private val relPos = Vector2d()
    private val relVel = Vector2d()

    private fun subDot(a: Vector2d, b: Vector2d, c: Vector2d): Double {
        return (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)
    }

    private fun distSqPointLineSegment(a: Vector2d, b: Vector2d, c: Vector2d): Double {
        val r = subDot(a, b, c) / b.distanceSquared(a)
        return when {
            r < 0.0 -> c.distanceSquared(a)
            r > 1.0 -> c.distanceSquared(b)
            else -> {
                val lx = a.x + r * (b.x - a.x)
                val ly = a.y + r * (b.y - a.y)
                c.distanceSquared(lx, ly)
            }
        }
    }

    private fun linearProgram1(
        lines: List<Line<Vector2d>>, lineNo: Int, radius: Double,
        optVelocity: Vector2d, directionOpt: Boolean, result: Vector2d
    ): Boolean {
        val line = lines[lineNo]
        val dot = line.point.dot(line.direction)
        val disc = dot * dot + radius * radius - line.point.lengthSquared()
        if (disc < 0.0) {
            // Max speed sphere fully invalidates line
            return false
        }
        val sqrtDisc = sqrt(disc)
        var tLeft = -dot - sqrtDisc
        var tRight = -dot + sqrtDisc

        for (i in 0 until lineNo) {
            val lineI = lines[i]
            val den = det(line.direction, lineI.direction)
            val num = det(lineI.direction, zero2d, line.point, lineI.point)
            if (abs(den) <= RVO_EPSILON) {
                // Lines line is (almost) parallel to plane i.
                if (num < 0.0) {
                    return false
                } else continue
            }
            val t = num / den
            if (den >= 0.0) {
                tRight = max(tRight, t)
            } else {
                tLeft = max(tLeft, t)
            }
            if (tLeft > tRight) {
                return false
            }
        }
        val factor = if (directionOpt) {
            // optimize direction
            // take right/left extreme
            if (optVelocity.dot(line.direction) > 0.0) tRight
            else tLeft
        } else {
            // optimize closest point
            val t = line.direction.dot(optVelocity) - line.direction.dot(line.point)
            clamp(t, tLeft, tRight)
        }
        // result = line.point + tRight * line.direction
        result.set(line.direction).mul(factor).add(line.point)
        return true
    }

    private fun linearProgram2(
        lines: List<Line<Vector2d>>, radius: Double,
        optVelocity: Vector2d, directionOpt: Boolean, result: Vector2d
    ): Int {
        when {
            directionOpt -> {
                // Optimize direction. Note that the optimization velocity is of unit length in this case.
                result.set(optVelocity).mul(radius)
            }
            optVelocity.lengthSquared() > sq(radius) -> {
                // Optimize closest point and outside circle
                result.set(optVelocity).mul(radius)
            }
            else -> {
                // Optimize closest point and inside circle
                result.set(optVelocity)
            }
        }
        for (i in lines.indices) {
            val lineI = lines[i]
            if (det(lineI.direction, zero2d, lineI.point, result) > 0.0) {
                // Result does not satisfy constraint i. Compute new optimal result
                val tmp = Vector2d(result)
                if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
                    result.set(tmp)
                    return i
                }
            }
        }
        return lines.size
    }

    private fun linearProgram3(
        lines: List<Line<Vector2d>>, numObstLines: Int, beginLine: Int,
        radius: Double, result: Vector2d
    ) {
        var distance = 0.0
        for (i in beginLine until lines.size) {
            val lineI = lines[i]
            val lineIPoint = lineI.point
            if (det(lineI.direction, zero2d, lineIPoint, result) > distance) {
                // Result does not satisfy constraint of line i
                val projLines = ArrayList<Line<Vector2d>>()
                projLines.addAll(lines.subList(0, numObstLines)) // awkward
                for (j in numObstLines until i) {
                    val line = Line(Vector2d(), Vector2d())
                    val linePoint = line.point
                    val lineJ = lines[j]
                    val lineJPoint = lineJ.point
                    val det = det(lineI.direction, lineJ.direction)
                    if (abs(det) <= RVO_EPSILON) {
                        // line i and j are parallel
                        if (lineI.direction.dot(lineJ.direction) > 0.0) {
                            // same direction
                            continue
                        } else {
                            // opposite directions
                            linePoint.set(lineIPoint).add(lineJPoint).mul(0.5)
                        }
                    } else {
                        linePoint.set(lineI.direction)
                            .mul(det(lineJ.direction, zero2d, lineIPoint, lineJPoint) / det)
                            .add(lineIPoint)
                    }
                    line.direction.set(lineJ.direction).sub(lineI.direction).normalize()
                    projLines.add(line)
                }
                val tmp = Vector2d(result)
                if (linearProgram2(
                        projLines, radius,
                        Vector2d(-lineI.direction.y, lineI.direction.x), true, result
                    ) < projLines.size
                ) {
                    /* This should in principle not happen. The result is by definition
					 * already in the feasible region of this linear program. If it fails,
					 * it is due to small floating point error, and the current result is
					 * kept.
					 */
                    result.set(tmp)
                }
                distance = det(lineI.direction, zero2d, lineIPoint, result)
            }
        }
    }

}