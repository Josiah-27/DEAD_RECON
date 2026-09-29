package com.deadrecon.app.nav

import kotlin.math.PI
import kotlin.math.pow

/**
 * Peak/valley step detector on the low-passed accelerometer magnitude, with Weinberg stride
 * estimation:  L = K · (a_max − a_min)^¼ .  K is user-calibratable ("walk a known 10 m").
 */
class StepDetector(var weinbergK: Double = 0.47) {

    data class Step(val timestampNs: Long, val length: Double, val interval: Double)

    private enum class Phase { PEAK, VALLEY }

    private var lp = 9.80665
    private var lastT = 0L
    private var phase = Phase.PEAK
    private var candPeak = 0.0
    private var candT = 0L
    private var valleyMin = Double.MAX_VALUE
    private var phaseStartT = 0L
    private var lastStepT = 0L
    private val recent = ArrayDeque<Long>()

    var peakThreshold = 10.55    // m/s² — peak must exceed ≈ g + 0.75
    var valleyThreshold = 9.55   // m/s² — must dip below this before the next peak

    fun reset() {
        lp = 9.80665; lastT = 0L; phase = Phase.PEAK; candPeak = 0.0
        valleyMin = Double.MAX_VALUE; lastStepT = 0L; recent.clear()
    }

    fun add(tNs: Long, accNorm: Double): Step? {
        if (lastT == 0L) { lastT = tNs; phaseStartT = tNs; lp = accNorm; return null }
        val dt = ((tNs - lastT) / 1e9).coerceIn(1e-4, 0.1)
        lastT = tNs
        val rc = 1.0 / (2 * PI * 3.0)          // 3 Hz low-pass
        lp += dt / (rc + dt) * (accNorm - lp)
        if (lp < valleyMin) valleyMin = lp

        while (recent.isNotEmpty() && tNs - recent.first() > 3_000_000_000L) recent.removeFirst()

        when (phase) {
            Phase.PEAK -> {
                if (lp > peakThreshold) {
                    if (lp > candPeak) { candPeak = lp; candT = tNs }
                } else if (candPeak > 0.0 && lp < candPeak - 0.45) {
                    val sinceLast = (candT - lastStepT) / 1e9
                    val result = if (lastStepT == 0L || sinceLast >= 0.28) {
                        val swing = (candPeak - valleyMin).coerceAtLeast(0.2)
                        val length = (weinbergK * swing.pow(0.25)).coerceIn(0.25, 1.5)
                        val interval = if (lastStepT == 0L || sinceLast > 2.0) 0.55 else sinceLast
                        lastStepT = candT
                        recent.addLast(candT)
                        Step(candT, length, interval)
                    } else null
                    candPeak = 0.0
                    valleyMin = lp
                    phase = Phase.VALLEY; phaseStartT = tNs
                    return result
                }
            }
            Phase.VALLEY -> {
                if (lp < valleyThreshold || (tNs - phaseStartT) > 1_500_000_000L) {
                    phase = Phase.PEAK; phaseStartT = tNs
                }
            }
        }
        return null
    }

    /** Steps per second over the last 3 s. */
    fun cadence(nowNs: Long): Double {
        while (recent.isNotEmpty() && nowNs - recent.first() > 3_000_000_000L) recent.removeFirst()
        return recent.size / 3.0
    }

    fun secondsSinceLastStep(nowNs: Long): Double =
        if (lastStepT == 0L) Double.MAX_VALUE else (nowNs - lastStepT) / 1e9
}
