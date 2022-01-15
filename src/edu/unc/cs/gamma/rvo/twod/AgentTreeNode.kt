package edu.unc.cs.gamma.rvo.twod

import org.joml.Vector2d

class AgentTreeNode {
    var begin = 0
    var end = 0
    var left = 0
    var right = 0
    val maxCoord = Vector2d()
    val minCoord = Vector2d()
    val delta = Vector2d()
}