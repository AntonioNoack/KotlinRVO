package edu.unc.cs.gamma.rvo.threed

import edu.unc.cs.gamma.rvo.Line
import edu.unc.cs.gamma.rvo.Plane
import edu.unc.cs.gamma.rvo.Ref
import edu.unc.cs.gamma.rvo.Utils.RVO_EPSILON
import edu.unc.cs.gamma.rvo.Utils.clamp
import edu.unc.cs.gamma.rvo.Utils.sq
import org.joml.Vector3d
import kotlin.math.max
import kotlin.math.sqrt

class Agent(
    val sim: RVOSimulator,
    val position: Vector3d,
    val maxNeighbors: Int,
    val maxSpeed: Double,
    val neighborDist: Double,
    val radius: Double,
    val timeHorizon: Double,
    val velocity: Vector3d,
    val id: Int
) {

    class Neighbor(var distSq: Double, var agent: Agent) {
        fun set(other: Neighbor) {
            distSq = other.distSq
            agent = other.agent
        }
    }

    val targetVelocity = Vector3d()
    val newVelocity = Vector3d()

    val neighbors = ArrayList<Neighbor>()

    private val orcaPlanes = ArrayList<Plane<Vector3d>>()

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
            sim.timeStep * velocity.y,
            sim.timeStep * velocity.z
        )
    }

    fun computeNeighbors() {
        neighbors.clear()
        if (maxNeighbors > 0) {
            sim.kdTree.computeAgentNeighbors(this, neighborDist * neighborDist)
        }
    }

    private fun linearProgram1(
        planes: List<Plane<Vector3d>>, planeNo: Int, line: Line<Vector3d>, radius: Double,
        optVelocity: Vector3d, directionOpt: Boolean,
        result: Vector3d
    ): Boolean {
        val dot = line.point.dot(line.direction)
        val disc = dot * dot + radius * radius - line.point.lengthSquared()
        if (disc < 0.0) {
            // Max speed sphere fully invalidates line
            return false
        }
        val sqrtDisc = sqrt(disc)
        var tLeft = -dot - sqrtDisc
        var tRight = -dot + sqrtDisc

        for (planeIndex in 0 until planeNo) {
            val plane = planes[planeIndex]
            val num = plane.point.dot(plane.normal) - line.point.dot(plane.normal)
            val den = line.direction.dot(plane.normal)
            if (den * den <= RVO_EPSILON) {
                // Lines line is (almost) parallel to plane i.
                if (num > 0.0) {
                    return false
                } else continue
            }
            val t = num / den
            if (den >= 0.0) {
                tLeft = max(tLeft, t)
            } else {
                tRight = max(tRight, t)
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
        planes: List<Plane<Vector3d>>, planeNo: Int, radius: Double, optVelocity: Vector3d,
        directionOpt: Boolean, result: Vector3d
    ): Boolean {
        val plane = planes[planeNo]
        val planePoint = plane.point
        val planeNormal = plane.normal
        val planeDist = planePoint.dot(planeNormal)
        val planeDistSq = sq(planeDist)
        val radiusSq = sq(radius)
        if (planeDistSq > radiusSq) {
            return false
        }
        val planeRadiusSq = radiusSq - planeDistSq
        val planeCenter = Vector3d(planeNormal).mul(planeDist)
        if (directionOpt) {
            // project direction optVelocity on plane
            val tmp = optVelocity.dot(planeNormal)
            val planeOptVelocity = Vector3d(optVelocity).sub(
                tmp * planeNormal.x,
                tmp * planeNormal.y,
                tmp * planeNormal.z
            )
            val povs = planeOptVelocity.lengthSquared()
            result.set(planeCenter)
            if (povs > RVO_EPSILON) {
                val f = sqrt(planeRadiusSq / povs)
                result.set(planeCenter)
                    .add(
                        f * planeOptVelocity.x,
                        f * planeOptVelocity.y,
                        f * planeOptVelocity.z
                    )
            }
        } else {
            // project point optVelocity on plane
            val f = planePoint.dot(planeNormal) - optVelocity.dot(planeNormal)
            result.set(planeNormal).mul(f)
                .add(optVelocity)
            // if outside planeCircle, project on planeCircle
            if (result.lengthSquared() > radiusSq) {
                val prl = result.distanceSquared(planeCenter)
                result.lerp(planeCenter, 1.0 - sqrt(planeRadiusSq / prl))
            }
        }

        val crossProduct = Vector3d()
        val line = Line(direction = crossProduct, point = Vector3d())
        for (i in 0 until planeNo) {
            val planeI = planes[i]
            val planeINormal = planeI.normal
            val planeIPoint = planeI.point
            if (planeINormal.dot(planeIPoint) > planeINormal.dot(result)) {
                // result does not satisfy constraint i. compute new optimal result
                // compute intersection line of plane i and plane planeNo
                crossProduct.set(planeINormal).cross(planeNormal)
                if (crossProduct.lengthSquared() <= RVO_EPSILON) {
                    // planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo
                    return false
                }
                crossProduct.normalize()
                intersectionMagic(
                    crossProduct, plane, planeI,
                    line.point
                )
                if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt, result)) {
                    return false
                }
            }
        }
        return true
    }

    private fun linearProgram3(
        planes: List<Plane<Vector3d>>, radius: Double, optVelocity: Vector3d,
        directionOpt: Boolean, result: Vector3d
    ): Int {
        when {
            directionOpt -> result.set(optVelocity).mul(radius)
            optVelocity.lengthSquared() > radius * radius -> result.set(optVelocity).normalize(radius)
            else -> result.set(optVelocity)
        }
        for (i in planes.indices) {
            val plane = planes[i]
            if (plane.normal.dot(plane.point) > plane.normal.dot(result)) {
                // result does not satisfy constraint i. compute new optimal result
                val tmpResult = Vector3d()
                if (!linearProgram2(planes, i, radius, optVelocity, directionOpt, result)) {
                    result.set(tmpResult)
                    return i
                }
            }
        }
        return planes.size
    }

    private fun linearProgram4(
        planes: List<Plane<Vector3d>>,
        beginPlane: Int, radius: Double, result: Vector3d
    ) {
        var distance = 0.0
        for (i in beginPlane until planes.size) {
            val planeI = planes[i]
            if (planeI.normal.dot(planeI.point) - planeI.normal.dot(result) > distance) {
                // result does not satisfy constraint of plane i
                val projPlanes = ArrayList<Plane<Vector3d>>()
                for (j in 0 until i) {
                    val plane = Plane(Vector3d(), Vector3d())
                    val planeJ = planes[j]
                    val crossProduct = Vector3d(planeJ.normal).cross(planeI.normal)
                    if (crossProduct.lengthSquared() <= RVO_EPSILON) {
                        // Plane i and plane j are (almost) parallel
                        if (planeI.normal.dot(planeJ.normal) > 0.0) {
                            // Plane i and plane j point in the same direction
                            continue
                        } else {
                            // Plane i and plane j point in opposite direction
                            plane.point
                                .set(planeI.point)
                                .add(planeJ.point)
                                .mul(0.5)
                        }
                    } else {
                        // Plane.point is point on line of intersection between plane i and plane j
                        intersectionMagic(
                            crossProduct,
                            planeI, planeJ,
                            plane.point
                        )
                    }
                    plane.normal.set(planeJ.normal).sub(planeI.normal).normalize()
                    projPlanes.add(plane)
                }
                val tmp = tmp.set(result)
                if (linearProgram3(projPlanes, radius, planeI.normal, true, result) < projPlanes.size) {
                    result.set(tmp)
                }
                distance = planeI.normal.dot(planeI.point) - planeI.normal.dot(result)
            }
        }
    }

    private val tmp = Vector3d()

    private fun intersectionMagic(
        crossProduct: Vector3d,
        planeI: Plane<Vector3d>, // or planeNo
        planeJ: Plane<Vector3d>, // or i
        planePoint: Vector3d
    ) {
        /*
        * const Vector3 lineNormal = cross(line.direction, planes[planeNo].normal);
		* const float f = ((planes[i].point - planes[planeNo].point) * planes[i].normal) / (lineNormal * planes[i].normal);
		* line.point = planes[planeNo].point + f * lineNormal;
        * */
        /*
        * const Vector3 lineNormal = cross(crossProduct, planes[i].normal);
        * double f = ((planes[j].point - planes[i].point) * planes[j].normal) / (lineNormal * planes[j].normal)
		* plane.point = planes[i].point + f * lineNormal;
        * */
        val lineNormal = planePoint.set(crossProduct).cross(planeI.normal)
        val f = (planeJ.point.dot(planeJ.normal) - planeI.point.dot(planeJ.normal)) / lineNormal.dot(planeJ.normal)
        planePoint.mul(f).add(planeI.point)
    }

    fun insertNeighbor(agent: Agent, rangeSq: Ref<Double>) {
        if (this !== agent) {
            val distDq = position.distanceSquared(agent.position)
            if (distDq < rangeSq.value) {
                if (neighbors.size < maxNeighbors) {
                    neighbors.add(Neighbor(distDq, agent))
                }
                var i = neighbors.lastIndex
                while (i != 0 && distDq < neighbors[i].distSq) {
                    neighbors[i].set(neighbors[i - 1]) // ??...
                    i--
                }
                neighbors[i].distSq = distDq
                neighbors[i].agent = agent
                if (neighbors.size == maxNeighbors) {
                    rangeSq.value = neighbors.last().distSq
                }
            }
        }
    }

    private val relVelocity = Vector3d()
    private val relPosition = Vector3d()

    fun computeNewVelocity() {
        while (orcaPlanes.size > neighbors.size) orcaPlanes.removeAt(orcaPlanes.lastIndex)
        while (orcaPlanes.size < neighbors.size) orcaPlanes.add(Plane(Vector3d(), Vector3d()))
        val invTimeHorizon = 1.0 / timeHorizon
        // create agent orca planes
        val relativePosition = relPosition
        val relativeVelocity = relVelocity
        for (i in neighbors.indices) {
            val other = neighbors[i].agent
            relativePosition.set(other.position).sub(position)
            relativeVelocity.set(velocity).sub(other.velocity)
            val distSq = relativePosition.lengthSquared()
            val combinedRadius = radius + other.radius
            val combinedRadiusSq = sq(combinedRadius)
            val plane = orcaPlanes[i]
            val w = plane.point
            val t = if (distSq > combinedRadius) {
                // no collision
                w.set(relativePosition).mul(-invTimeHorizon).add(relativeVelocity)
                // vector from cutoff center to relative velocity
                val wLengthSq = w.lengthSquared()
                val dot = w.dot(relativePosition)
                if (dot < 0.0 && dot * dot > combinedRadiusSq * wLengthSq) {
                    // project on cut-off circle
                    invTimeHorizon
                } else {
                    // project on cone
                    val a = distSq + 0.0
                    val b = relativePosition.dot(relativeVelocity)
                    // absSq(relativeVelocity) - absSq(cross(relativePosition, relativeVelocity)) / (distSq - combinedRadiusSq)
                    val lengthSq2 = crossLengthSquared(relativePosition, relativeVelocity) / (distSq - combinedRadiusSq)
                    val c = relativeVelocity.lengthSquared() - lengthSq2
                    // todo somewhere here must be an error, because I had to add max(0,x) to sqrt
                    val t = (b + sqrt(max(0.0, b * b - a * c))) / a
                    w.set(relativePosition).mul(-t).add(relativeVelocity)
                    // println("bb $t -> $w by $distSq, $combinedRadiusSq, $lengthSq2, $c")
                    t
                }
            } else {
                // collision / intersection
                val invTimeStep = 1.0 / sim.timeStep
                w.set(relativePosition).mul(-invTimeStep).add(relativeVelocity)
                invTimeStep
            }
            val wLength = w.length()
            w.div(wLength)
            plane.normal.set(w)
            w.mul((combinedRadius * t - wLength) * 0.5)
            w.add(velocity)
            // orcaPlanes.add(plane), already added
        }
        val planeFail = linearProgram3(orcaPlanes, maxSpeed, targetVelocity, false, newVelocity)
        if (planeFail < orcaPlanes.size) {
            linearProgram4(orcaPlanes, planeFail, maxSpeed, newVelocity)
        }
    }

    private fun crossLengthSquared(a: Vector3d, v: Vector3d): Double {
        val rx = a.y * v.z - a.z * v.y
        val ry = a.z * v.x - a.x * v.z
        val rz = a.x * v.y - a.y * v.x
        return rx * rx + ry * ry + rz * rz
    }

}