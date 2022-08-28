package org.firstinspires.ftc.teamcode.wrappers

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utils.TAU

class AGEncoder(config: String, private val ticksPerRev: Double, private val gearRatio: Double = 1.0, hmap: HardwareMap) {
    private val encoder = hmap.get(DcMotorEx::class.java, config)

    private var position = 0.0
    private var lastPosition = 0.0

    val currentPos: Double
        get() {
            val pos = encoder.currentPosition.toDouble()
            position += (pos - lastPosition)
            lastPosition = pos
            return position
        }

    val rotations: Double
        get() = currentPos / (ticksPerRev * gearRatio)
    val radians: Double
        get() = rotations * TAU

    fun reverse() {
        encoder.direction = DcMotorSimple.Direction.REVERSE
    }

    infix fun reset(newPosition: Double) {
        lastPosition = encoder.currentPosition.toDouble()
        position = newPosition
    }
}