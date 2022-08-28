package org.firstinspires.ftc.teamcode.lib

import com.acmerobotics.dashboard.*
import com.acmerobotics.dashboard.telemetry.*
import org.firstinspires.ftc.robotcore.external.*

class TelemetryBuilder(private val telemetry: Telemetry) {
    var packet = TelemetryPacket()
        private set

    fun ready(): TelemetryBuilder {
        add("ready")
        update()
        return this
    }

    fun add(line: String?): TelemetryBuilder {
        packet.addLine(line)
        telemetry.addLine(line)
        return this
    }

    fun add(key: String?, value: Any?): TelemetryBuilder {
        packet.put(key, value)
        telemetry.addData(key, value)
        return this
    }

    fun update(): TelemetryBuilder {
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
        packet = TelemetryPacket()
        telemetry.update()
        return this
    }
}