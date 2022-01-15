package edu.unc.cs.gamma.rvo.threed

import org.joml.Vector3d

class AgentTreeNode {
    var begin = 0
    var end = 0
    var left = 0
    var right = 0
    val maxCoord = Vector3d()
    val minCoord = Vector3d()
    val delta = Vector3d()
}