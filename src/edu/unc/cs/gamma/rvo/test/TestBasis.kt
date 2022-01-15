package edu.unc.cs.gamma.rvo.test

import edu.unc.cs.gamma.rvo.Utils
import org.joml.Vector2d
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.image.BufferedImage
import java.io.File
import java.util.*
import javax.imageio.ImageIO
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

fun getAngle(i: Int, numAgents: Int): Double {
    return i * Math.PI * 2.0 / numAgents
}

fun generateColors(numAgents: Int): IntArray {
    return IntArray(numAgents) { i ->
        val angle = getAngle(i, numAgents)
        (cos(angle) * 127 + 127).toInt().shl(16) + (sin(angle) * 127 + 127).toInt()
    }
}

fun mix(a: Int, b: Int, f: Double): Int {
    return (a * (1.0 - f) + b * f).toInt()
}

fun mix255(a: Int, b: Int, f: Double): Int {
    return mix(a.and(255), b.and(255), f)
}

fun mixARGB(a: Int, b: Int, f: Double): Int {
    return mix255(a.shr(16), b.shr(16), f).shl(16) +
            mix255(a.shr(8), b.shr(8), f).shl(8) +
            mix255(a, b, f)
}

fun <Simulator, Agent> runTest(
    outputPrefix: String,
    printNumbers: Boolean,
    createRVO: (timeStep: Double, radius: Double, speed: Double) -> Simulator,
    addAgent: (sim: Simulator, x: Double, y: Double) -> Agent,
    getDistanceSquared: (agent: Agent, x: Double, y: Double) -> Double,
    updateTargetVelocity: (agent: Agent, x: Double, y: Double) -> Unit,
    step: (sim: Simulator) -> Unit,
    getAgentX: (agent: Agent) -> Double,
    getAgentY: (agent: Agent) -> Double,
) {

    val imageCount = 100
    val density = 0.5 // [0,1]
    val radius = 1.0
    val sceneRadius = 20.0
    val numAgents = max(2, (sceneRadius / radius * Math.PI * density).toInt())

    val colors = generateColors(numAgents)
    val targets = Array(numAgents) { Vector2d() }
    val agents = ArrayList<Agent>(numAgents)

    val speed = 1.0

    val timeStep = 0.5
    val randomness = 0.5

    val extraDurationFactor = 2.0

    val rvo = createRVO(timeStep, radius, speed)

    val random = Random(1234L)

    for (i in 0 until numAgents) {
        val angle = Math.PI * 2 * i / numAgents
        val rx = (random.nextDouble() - 0.5) * randomness
        val ry = (random.nextDouble() - 0.5) * randomness
        val px = sceneRadius * cos(angle) + rx
        val py = sceneRadius * sin(angle) + ry
        agents.add(addAgent(rvo, px, py))
        targets[i].set(-px, -py)
    }

    var frameCtr = 0

    val home = System.getenv("user.home")
    val folder = File(home, "Documents/agents/")
    if (!folder.exists()) folder.mkdirs()

    fun printFrame() {
        val w = 512
        val h = 512
        val scale = (sceneRadius * 2.5) / w
        val image = BufferedImage(w, h, 1)
        val raster = image.raster
        val buffer = raster.dataBuffer
        val smoothness = 0.5 * scale
        val invRadiusSq = 1.0 / Utils.sq(radius)
        val minDistSq = Utils.sq(1.0 - smoothness)
        val maxDistSq = Utils.sq(1.0 + smoothness)
        // could be optimized, but it's a test only anyways
        for (y in 0 until h) {
            for (x in 0 until w) {
                var pixelColor = -1
                val xi = (x - w / 2) * scale
                val yi = (y - h / 2) * scale
                for (i in 0 until numAgents) {
                    val distSq = getDistanceSquared(agents[i], xi, yi) * invRadiusSq
                    if (distSq < maxDistSq) {
                        pixelColor = if (distSq < minDistSq) {
                            colors[i]
                        } else {
                            val f = (1.0 - sqrt(distSq)) * 0.5 / smoothness + 0.5
                            mixARGB(pixelColor, colors[i], f)
                        }
                    }
                }
                buffer.setElem(x + y * w, pixelColor)
            }
        }
        if(printNumbers){
            val g = image.graphics as Graphics2D
            g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON)
            val metrics = g.getFontMetrics(g.font)
            for (i in agents.indices) {
                val agent = agents[i]
                val text = i.toString()
                val cx = -metrics.stringWidth(text) * 0.5
                val cy = -metrics.height * 0.5 + metrics.ascent
                g.drawString(
                    text,
                    (getAgentX(agent) / scale + w / 2 - cx).toFloat(),
                    (getAgentY(agent) / scale + h / 2 - cy).toFloat()
                )
            }
            g.dispose()
        }
        ImageIO.write(image, "png", File(folder, "${outputPrefix}-${frameCtr++}.png"))
    }

    val pathLength = 2.0 * sceneRadius
    val expectedDuration = extraDurationFactor * pathLength / speed

    val expectedSteps = (expectedDuration / timeStep).toInt()

    var lastJ = -1
    for (timeStepIndex in 0 until expectedSteps) {
        val j = (timeStepIndex * imageCount) / expectedSteps
        if (j != lastJ) {
            lastJ = j
            printFrame()
        }
        // update agent velocities
        for (i in 0 until numAgents) {
            val target = targets[i]
            updateTargetVelocity(agents[i], target.x, target.y)
        }
        step(rvo)
    }

    printFrame()

}