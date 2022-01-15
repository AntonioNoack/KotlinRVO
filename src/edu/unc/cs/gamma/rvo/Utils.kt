package edu.unc.cs.gamma.rvo

import org.joml.Vector2d
import org.joml.Vector3d

object Utils {

    fun sq(f: Double) = f * f

    var RVO_EPSILON = 0.00001
    var RVO_MAX_LEAF_SIZE = 10

    fun clamp(v: Double, min: Double, max: Double): Double {
        return if (v < min) min else if (v < max) v else max
    }

    fun det(fa: Double, a: Vector2d, b: Vector2d, c: Vector2d, d: Vector2d): Double {
        return det(fa * a.x - b.x, fa * a.y - b.y, c.x - d.x, c.y - d.y)
    }

    fun det(a: Vector2d, b: Vector2d, c: Vector2d, d: Vector2d): Double {
        return det(a.x - b.x, a.y - b.y, c.x - d.x, c.y - d.y)
    }

    fun det(a: Vector2d, b: Vector2d): Double {
        return det(a.x, a.y, b.x, b.y)
    }

    fun det(a: Double, b: Double, c: Double, d: Double): Double {
        return a * d - b * c
    }

    fun leftOf(a: Vector2d, b: Vector2d, c: Vector2d): Double {
        return det(a, c, b, a)
    }

    // must not be changed!!
    val zero2d = Vector2d()
    val zero3d = Vector3d()

}