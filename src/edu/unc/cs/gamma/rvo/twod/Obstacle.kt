package edu.unc.cs.gamma.rvo.twod

import org.joml.Vector2d

class Obstacle(
    var isConvex: Boolean,
    var nextObstacle: Obstacle?,
    var prevObstacle: Obstacle?,
    var point: Vector2d,
    var unitDir: Vector2d,
    var id: Int,
)