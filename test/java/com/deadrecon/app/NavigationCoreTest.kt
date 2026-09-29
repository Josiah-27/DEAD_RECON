package com.deadrecon.app

import com.deadrecon.app.ai.MotionMode
import com.deadrecon.app.geo.GeoFormat
import com.deadrecon.app.geo.GeoMath
import com.deadrecon.app.geo.LatLon
import com.deadrecon.app.geo.LocalTangentPlane
import com.deadrecon.app.nav.NavigationEngine
import com.deadrecon.app.vision.OpticalFlowEngine
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.*

/** Pure-JVM tests for the fusion core: run with ./gradlew test (no device needed). */
class NavigationCoreTest {

    private fun texture(w: Int, h: Int, warp: (Double, Double) -> Pair<Double, Double>) = FloatArray(w * h) { i ->
        val (x, y) = warp((i % w).toDouble(), (i / w).toDouble())
        (0.5 + 0.2 * sin(x * 0.45) * cos(y * 0.37) + 0.15 * sin(x * 0.13 + y * 0.29) + 0.1 * cos(x * 0.71 - y * 0.23)).toFloat()
    }

    @Test fun geoRoundTripAndDistance() {
        val ltp = LocalTangentPlane(LatLon(12.0, 77.0))
        val p = ltp.toGeo(1000.0, 1000.0)
        val (e, n) = ltp.toLocal(p)
        assertEquals(1000.0, e, 0.05); assertEquals(1000.0, n, 0.05)
        assertEquals(1414.2, GeoMath.distance(LatLon(12.0, 77.0), p), 5.0)
        assertEquals(45.0, GeoMath.bearing(LatLon(12.0, 77.0), p), 0.5)
    }

    @Test fun coordinateParsing() {
        assertEquals(12.97161, GeoFormat.parse("12°58'17.8\"N", true)!!, 1e-4)
        assertEquals(-77.5946, GeoFormat.parse("77.5946 W", false)!!, 1e-9)
        assertEquals(null, GeoFormat.parse("95", true))
    }

    @Test fun opticalFlowTranslationAndExpansion() {
        val of = OpticalFlowEngine()
        of.process(texture(80, 60) { x, y -> x to y })
        val r = of.process(texture(80, 60) { x, y -> (x - 1.6) to (y + 0.8) })
        assertEquals(1.6, r.tx, 0.15); assertEquals(-0.8, r.ty, 0.15)
        of.reset()
        of.process(texture(80, 60) { x, y -> x to y })
        val z = of.process(texture(80, 60) { x, y -> (40 + (x - 40) / 1.04) to (30 + (y - 30) / 1.04) })
        assertEquals(0.04, z.expansion, 0.008)
        of.reset()
        of.process(FloatArray(4800) { 0.5f })
        val blank = of.process(FloatArray(4800) { 0.5f })
        assertTrue(!blank.valid || blank.confidence < 0.1)
    }

    @Test fun stationaryPhoneDoesNotDrift() {
        val eng = NavigationEngine(); val rnd = java.util.Random(1)
        var t = 1_000_000_000L
        repeat(12_000) {
            t += 10_000_000L
            eng.onGyroscope(t, 0.004 + rnd.nextGaussian() * 0.003, -0.003 + rnd.nextGaussian() * 0.003, rnd.nextGaussian() * 0.003)
            eng.onMagnetometer(t, 0.0, 30.0, -20.0)
            eng.onAccelerometer(t, 0.05 + rnd.nextGaussian() * 0.03, rnd.nextGaussian() * 0.03, 9.80665 + rnd.nextGaussian() * 0.03)
            if (it == 300) eng.setOriginHere()
        }
        assertTrue(eng.snapshot().displacement < 0.2)
    }

    @Test fun simulatedLWalk() {
        val eng = NavigationEngine(); val rnd = java.util.Random(3)
        var t = 1_000_000_000L; var heading = 0.0; var trueE = 0.0; var trueN = 0.0
        fun tick(walking: Boolean, turn: Double) {
            t += 10_000_000L; heading += turn * 0.01
            val ph = t / 1e9 * 2 * PI * 1.8
            eng.onGyroscope(t, rnd.nextGaussian() * 0.003, rnd.nextGaussian() * 0.003, -turn)
            eng.onMagnetometer(t, -30 * sin(heading), 30 * cos(heading), -20.0)
            eng.onAccelerometer(t, rnd.nextGaussian() * 0.04,
                (if (walking) cos(ph) else 0.0) + rnd.nextGaussian() * 0.04,
                9.80665 + (if (walking) 2.2 * sin(ph) else 0.0) + rnd.nextGaussian() * 0.04)
            if (walking) { trueE += 1.3 * sin(heading) * 0.01; trueN += 1.3 * cos(heading) * 0.01 }
        }
        repeat(300) { tick(false, 0.0) }
        eng.setOriginHere()
        repeat(1540) { tick(true, 0.0) }
        repeat(157) { tick(false, PI / 2 / 1.57) }
        repeat(770) { tick(true, 0.0) }
        repeat(500) { tick(false, 0.0) }
        val s = eng.snapshot()
        assertTrue("error too large", hypot(s.east - trueE, s.north - trueN) < 3.5)
        assertEquals(90.0, s.headingDeg, 5.0)
        assertEquals(MotionMode.STATIONARY, s.mode)
    }

    @Test fun demoWalkAndReturn() {
        val eng = NavigationEngine()
        eng.startDemo()
        repeat(1800) { eng.demoTick(0.05) }
        eng.startReturn()
        var arrived = false
        repeat(2400) { if (!arrived) { eng.demoTick(0.05); arrived = eng.snapshot().guidance.arrived } }
        assertTrue(arrived)
    }
}
