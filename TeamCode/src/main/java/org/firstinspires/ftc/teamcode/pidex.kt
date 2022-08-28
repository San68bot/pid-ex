package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.controllers.PID
import org.firstinspires.ftc.teamcode.lib.TelemetryBuilder
import org.firstinspires.ftc.teamcode.utils.toDegrees
import org.firstinspires.ftc.teamcode.utils.toRadians
import org.firstinspires.ftc.teamcode.wrappers.*

@TeleOp @Config
class pidex: LinearOpMode() {
    override fun runOpMode() {
        val actionTimer = ActionTimer()
        val telemetryBuilder = TelemetryBuilder(telemetry)

        val motor = AGMotor("motor", Go_312::class, 1.0, hardwareMap) {
            resetEncoder(0.0)
            float
            RUN_WITHOUT_ENCODER
        }
        val pid = PID(kP, kI, kD, 0.0, leeway.toRadians, integralThreshold.toRadians, actionTimer)

        telemetryBuilder.ready()
        waitForStart()
        actionTimer.reset()
        while (opModeIsActive()) {
            if (reset) {
                motor.resetEncoder(0.0)
                reset = false
            }

            val radians = motor.currentRadians()

            val power = pid
                .setConstants(kP, kI, kD, 0.0, leeway.toRadians, integralThreshold.toRadians)
                .target(target_theta.toRadians)
                .update(radians)

            if (act) {
                motor.power = power
            }

            telemetryBuilder
                .add("current radians", radians.toDegrees)
                .add("target radians", target_theta)
                .add("error", pid.error().toDegrees)
                .add("power", power)
                .update()
        }
    }

    companion object {
        @JvmField var act = false
        @JvmField var reset = false

        @JvmField var target_theta = 180.0

        @JvmField var kP: Double = 0.12
        @JvmField var kI: Double = 0.03
        @JvmField var kD: Double = 0.015
        @JvmField var leeway: Double = 0.0
        @JvmField var integralThreshold: Double = 30.0
    }
}