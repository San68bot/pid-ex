package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.Range
import kotlin.math.PI
import kotlin.math.absoluteValue

infix fun Double.difference(other: Double) = (this - other).absoluteValue

const val pi = Math.PI

const val EPSILON = 1e-6
infix fun Double.epsilonEquals(other: Double) = this difference other < EPSILON

infix fun Double.round(decimals: Int): Double {
    var multiplier = 1.0
    repeat(decimals) { multiplier *= 10 }
    return kotlin.math.round(this * multiplier) / multiplier
}

const val TAU = PI * 2.0
val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)