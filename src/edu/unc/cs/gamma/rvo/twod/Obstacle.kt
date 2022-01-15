package edu.unc.cs.gamma.rvo.twod

import org.joml.Vector2d

class Obstacle(
    var isConvex: Boolean,
    var nextObstacle: Obstacle?,
    var prevObstacle: Obstacle?,
    var point: Vector2d,
    var unitDir: Vector2d,
    var id: Int,
) {

    constructor() : this(false, null, null, Vector2d(), Vector2d(), 0)

}