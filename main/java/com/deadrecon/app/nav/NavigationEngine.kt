package com.deadrecon.app.nav

import com.deadrecon.app.ai.MotionClassifier
import com.deadrecon.app.ai.MotionMode
import com.deadrecon.app.map.DemoMap
import com.deadrecon.app.vision.OpticalFlowEngine
import kotlin.math.*

/** Immutable view of the navigation state, published to the UI ~20×/s. */
data class NavSnapshot(
    val aligned: Boolean = false,
    val east: Double = 0.0,
    val north: Double = 0.0,
    val velE: Double = 0.0,
    val velN: Double = 0.0,
    val sigmaE: Double = 0.0,
    val sigmaN: Double = 0.0,
    val covEN: Double = 0.0,
    val headingDeg: Double = 0.0,
    val mode: MotionMode = MotionMode.STATIONARY,
    val modeConfidence: Double = 0.0,
    val stepCount: Int = 0,
    val pathLength: Double = 0.0,
    val strideK: Double = 0.47,
    val calibrating: Boolean = false,
    val calibrationDistance: Double = 0.0,
    val visionConfidence: Double = 0.0,
    val visionActive: Boolean = false,
    val visionCalibrated: Boolean = false,
    val visionStill: Boolean = false,
    val visionVelE: Double = 0.0,
    val visionVelN: Double = 0.0,
    val visionDepth: Double = 0.0,
    val visionConsistency: Double = 0.0,
    val magReliability: Double = 0.0,
    val trajectory: List<Point2D> = listOf(Point2D(0.0, 0.0)),
    val explored: LongArray = LongArray(0),
    val exploredArea: Double = 0.0,
    val guidance: ReturnGuidance = ReturnGuidance(false),
    val demoActive: Boolean = false,
    val demoTruth: Point2D? = null,
) {
    val speed get() = hypot(velE, velN)
    val displacement get() = hypot(east, north)
}

/**
 * The whole sensor-fusion pipeline, free of Android types so it can be unit-tested on a JVM.
 * NOT thread-safe: call every method from the same (sensor) thread.
 *
 *  IMU ─► Orientation EKF ─► gravity removal ─► Position EKF ◄─ ZUPT
 *   │                                               ▲   ▲
 *   └► step detector ─► Weinberg stride ─► PDR ─────┘   │
 *   └► HMM motion classifier (selects models/updates)   │
 *  Camera ─► pyramidal LK ─► rotation-compensated, self-scaled visual velocity
 */
class NavigationEngine {
    val orientation = OrientationEkf()
    val position = PositionEkf()
    val steps = StepDetector()
    val classifier = MotionClassifier()
    private val returnNav = ReturnNavigator()

    var declinationDeg = 0.0

    // --- raw sensor bookkeeping
    private var startNs = 0L
    private var lastAccT = 0L
    private var lastGyroT = 0L
    private val lastAcc = DoubleArray(3)
    private val lastMag = DoubleArray(3)
    private var haveMag = false
    private val lastGyroRaw = DoubleArray(3)
    private val gyroRate = DoubleArray(3)
    private val accStats = RollingStats(150)
    private val gyroStats = RollingStats(150)
    private val strictAcc = RollingStats(30)
    private val strictGyro = RollingStats(30)
    private var stillSinceNs = 0L
    private var accCount = 0
    private var aligned = false
    private var headingDeg = 0.0

    // --- PDR
    private var stepCount = 0
    private var pathLength = 0.0
    private var lastStepT = 0L
    private var lastStepVelE = 0.0
    private var lastStepVelN = 0.0
    private var calibrating = false
    private var calLength = 0.0
    private var anchorE = 0.0
    private var anchorN = 0.0
    private var anchorT = 0L
    private val pending = ArrayList<PendingStep>()

    // --- vision
    private var logDepth = ln(1.5)
    private var visionConsistency = 0.0
    private var visionSamples = 0
    private var visionConf = 0.0
    private var visionActive = false
    private var visionCalibrated = false
    private var visionStillT = 0L
    private var lastVisionUpdateT = 0L
    private var visionVelE = 0.0
    private var visionVelN = 0.0

    // --- trajectory / coverage
    private val trajectory = ArrayList<Point2D>().apply { add(Point2D(0.0, 0.0)) }
    private val explored = HashSet<Long>()
    private var trajDirty = true
    private var cachedTraj: List<Point2D> = trajectory.toList()
    private var cachedExplored = LongArray(0)

    // --- demo walker
    var demoActive = false; private set
    private var demoE = 0.0
    private var demoN = 0.0
    private var demoIdx = 1
    private var demoStepAcc = 0.0
    private var demoClockNs = 0L
    private val rng = java.util.Random(7)
    private val demoSpeed = 1.3

    // ============================== sensor inputs ==============================

    fun onAccelerometer(t: Long, ax: Double, ay: Double, az: Double) {
        lastAcc[0] = ax; lastAcc[1] = ay; lastAcc[2] = az
        if (startNs == 0L) startNs = t
        if (!orientation.initialized) {
            if (haveMag || t - startNs > 1_500_000_000L) orientation.initialize(lastAcc, if (haveMag) lastMag else null)
            lastAccT = t
            return
        }
        val dt = if (lastAccT == 0L) 0.01 else (t - lastAccT) / 1e9
        lastAccT = t

        orientation.updateAccel(ax, ay, az)
        val norm = sqrt(ax * ax + ay * ay + az * az)
        accStats.add(norm); strictAcc.add(norm)
        val w = orientation.bodyToWorld(lastAcc)          // gravity is purely vertical → E/N are linear accel
        val step = steps.add(t, norm)

        if (++accCount % 5 == 0) runClassifier(t)
        if (!demoActive) headingDeg = (Math.toDegrees(orientation.travelBearing()) + 360) % 360
        if (demoActive) return

        val stationaryStrict = strictAcc.full && strictAcc.std < 0.12 && strictGyro.mean < 0.06
        stillSinceNs = if (stationaryStrict) (if (stillSinceNs == 0L) t else stillSinceNs) else 0L
        if (stillSinceNs != 0L && t - stillSinceNs > 800_000_000L) {
            orientation.learnGyroBias(lastGyroRaw[0], lastGyroRaw[1], lastGyroRaw[2])
        }

        val mode = classifier.mode
        val walking = mode == MotionMode.WALKING || mode == MotionMode.RUNNING
        val sigma = when (mode) {
            MotionMode.STATIONARY -> 0.3; MotionMode.WALKING -> 1.0; MotionMode.RUNNING -> 1.6
            MotionMode.CRAWLING -> 0.8; MotionMode.HANDLING -> 0.6
        }
        position.predict(w[0], w[1], dt, sigma, useAccel = !walking)

        if (step != null && aligned) {
            val gaitP = classifier.probability(MotionMode.WALKING) + classifier.probability(MotionMode.RUNNING)
            val confirmed = walking || gaitP > 0.35 || steps.cadence(t) >= 1.0
            applyStep(step.length, step.interval, orientation.travelBearing(), t, confirmed)
        }

        val visionStill = t - visionStillT < 300_000_000L
        val zupt = stationaryStrict ||
                classifier.probability(MotionMode.STATIONARY) > 0.75 ||
                (visionStill && strictAcc.std < 0.35 && strictGyro.mean < 0.25)
        if (zupt) position.zupt(0.01)

        recordTrajectory()
    }

    fun onGyroscope(t: Long, gx: Double, gy: Double, gz: Double) {
        lastGyroRaw[0] = gx; lastGyroRaw[1] = gy; lastGyroRaw[2] = gz
        if (lastGyroT != 0L && orientation.initialized) orientation.predict(gx, gy, gz, (t - lastGyroT) / 1e9)
        lastGyroT = t
        val b = orientation.gyroBias
        gyroRate[0] = gx - b[0]; gyroRate[1] = gy - b[1]; gyroRate[2] = gz - b[2]
        val n = sqrt(gyroRate[0].pow(2) + gyroRate[1].pow(2) + gyroRate[2].pow(2))
        gyroStats.add(n); strictGyro.add(n)
    }

    fun onMagnetometer(@Suppress("UNUSED_PARAMETER") t: Long, mx: Double, my: Double, mz: Double) {
        lastMag[0] = mx; lastMag[1] = my; lastMag[2] = mz; haveMag = true
        if (orientation.initialized) orientation.updateMag(mx, my, mz, Math.toRadians(declinationDeg))
    }

    /**
     * @param frameDt       seconds between this and the previous camera frame
     * @param rotationDeg   ImageInfo.rotationDegrees (image → upright display, clockwise)
     */
    fun onVisionFrame(r: OpticalFlowEngine.Result, frameDt: Double, rotationDeg: Int, focalPx: Double) {
        val t = lastAccT
        visionConf += 0.3 * ((if (r.valid) r.confidence else 0.0) - visionConf)
        if (!r.valid || frameDt <= 0.0 || frameDt > 0.2 || !orientation.initialized) { visionActive = false; return }

        // image axes → upright display axes (x right, y down) → body axes (x right, y up)
        val (dx, dy) = rotateCw(r.tx, r.ty, rotationDeg)
        var bx = dx / frameDt
        var by = -dy / frameDt
        // remove the flow a pure rotation of the camera would have produced
        bx -= focalPx * gyroRate[1]
        by += focalPx * gyroRate[0]
        val sRate = r.expansion / frameDt
        // camera looks along body −Z: lateral flow ↔ X/Y motion, expansion ↔ motion along −Z
        val unit = doubleArrayOf(-bx / focalPx, -by / focalPx, -sRate)   // velocity per metre of depth
        val w = orientation.bodyToWorld(unit)
        val uh = hypot(w[0], w[1])

        if (r.confidence > 0.35 && uh < 0.03 && abs(sRate) < 0.03) visionStillT = t

        // Self-supervised scale: learn scene depth from agreement with step-based velocity.
        val pdrSpeed = hypot(lastStepVelE, lastStepVelN)
        if (t - lastStepT < 1_000_000_000L && pdrSpeed > 0.3 && r.confidence > 0.35 && uh > 0.02) {
            val cos = (w[0] * lastStepVelE + w[1] * lastStepVelN) / (uh * pdrSpeed)
            visionConsistency += 0.05 * (cos - visionConsistency)
            if (cos > 0.5) {
                logDepth += 0.05 * (ln(pdrSpeed / uh) - logDepth)
                logDepth = logDepth.coerceIn(ln(0.3), ln(12.0))
                visionSamples++
            }
        }
        visionCalibrated = visionSamples >= 25 && visionConsistency > 0.55
        val depth = exp(logDepth)
        visionVelE = depth * w[0]; visionVelN = depth * w[1]

        visionActive = visionCalibrated && r.confidence > 0.3 &&
                classifier.mode != MotionMode.HANDLING && !demoActive
        if (visionActive) {
            val sigma = 0.12 + 0.6 * (1.0 - r.confidence)
            if (position.updateVelocity(visionVelE, visionVelN, sigma, gate = 9.21)) lastVisionUpdateT = t
        }
    }

    // ============================== commands ==============================

    /** Current position becomes (0,0) — the geographic start point. */
    fun setOriginHere() {
        position.resetPosition()
        trajectory.clear(); trajectory.add(Point2D(0.0, 0.0))
        explored.clear(); markExplored(0.0, 0.0)
        returnNav.stop()
        stepCount = 0; pathLength = 0.0
        anchorT = 0L; pending.clear()
        trajDirty = true
    }

    fun resetAll() {
        orientation.reset(); position.reset(); steps.reset(); classifier.reset()
        returnNav.stop(); demoActive = false
        startNs = 0L; lastAccT = 0L; lastGyroT = 0L; aligned = false; accCount = 0
        accStats.clear(); gyroStats.clear(); strictAcc.clear(); strictGyro.clear()
        logDepth = ln(1.5); visionConsistency = 0.0; visionSamples = 0; visionCalibrated = false
        calibrating = false
        setOriginHere()
    }

    fun startReturn() { returnNav.start(trajectory.toList()) }
    fun stopReturn() { returnNav.stop() }
    val returning get() = returnNav.active

    fun startCalibration() { calibrating = true; calLength = 0.0 }

    /** Finish "walk a known distance" calibration. Returns the new Weinberg K, or null. */
    fun finishCalibration(knownDistance: Double): Double? {
        calibrating = false
        if (calLength < 3.0) return null
        val k = (steps.weinbergK * knownDistance / calLength).coerceIn(0.30, 0.75)
        steps.weinbergK = k
        return k
    }

    fun startDemo() {
        demoActive = true
        position.reset()
        setOriginHere()
        demoE = 0.0; demoN = 0.0; demoIdx = 1; demoStepAcc = 0.0
    }

    fun stopDemo() { demoActive = false; returnNav.stop() }

    /** Advances the synthetic walker; its steps go through the real PDR → EKF path. */
    fun demoTick(dt: Double) {
        if (!demoActive) return
        demoClockNs += (dt * 1e9).toLong()
        val route = DemoMap.demoRoute
        var dx: Double; var dy: Double; var d: Double
        val moving: Boolean
        if (returnNav.active) {
            // Like a real rescuer: follow the on-screen arrow, which is computed from the *estimate*.
            val g = returnNav.update(position.east, position.north, headingDeg)
            dx = g.targetE - position.east; dy = g.targetN - position.north; d = hypot(dx, dy)
            moving = !g.arrived && d > 0.05
        } else {
            dx = route[demoIdx].x - demoE; dy = route[demoIdx].y - demoN; d = hypot(dx, dy)
            if (d < 0.3 && demoIdx < route.lastIndex) {
                demoIdx++
                dx = route[demoIdx].x - demoE; dy = route[demoIdx].y - demoN; d = hypot(dx, dy)
            }
            moving = d > 0.3
        }
        if (moving) {
            val stepLen = if (returnNav.active) demoSpeed * dt else min(demoSpeed * dt, d)
            demoE += dx / d * stepLen; demoN += dy / d * stepLen
            demoStepAcc += stepLen
            headingDeg = (Math.toDegrees(atan2(dx, dy)) + 360) % 360
        }
        position.predict(0.0, 0.0, dt, 1.0, useAccel = false)
        if (demoStepAcc >= 0.72) {
            val len = demoStepAcc * (1 + 0.03 * rng.nextGaussian())
            val hdg = Math.toRadians(headingDeg + 2.0 * rng.nextGaussian())
            applyStep(len, 0.72 / demoSpeed, hdg, demoClockNs)
            demoStepAcc = 0.0
        }
        if (!moving) position.zupt(0.01)
        recordTrajectory()
    }

    // ============================== output ==============================

    fun snapshot(): NavSnapshot {
        if (trajDirty) {
            cachedTraj = trajectory.toList()
            cachedExplored = explored.toLongArray()
            trajDirty = false
        }
        val g = returnNav.update(position.east, position.north, headingDeg)
        val mode = if (demoActive) (if (demoStepAcc > 0 || hypot(position.velE, position.velN) > 0.2) MotionMode.WALKING else MotionMode.STATIONARY) else classifier.mode
        // The step-anchored EKF covariance is locally consistent but ignores slowly-accumulating
        // stride-scale and heading bias, so add an empirical PDR drift term (~3 % of distance).
        val drift2 = (DRIFT_FRACTION * pathLength).pow(2)
        return NavSnapshot(
            aligned = aligned || demoActive,
            east = position.east, north = position.north,
            velE = position.velE, velN = position.velN,
            sigmaE = sqrt(position.P[0][0] + drift2), sigmaN = sqrt(position.P[1][1] + drift2), covEN = position.P[0][1],
            headingDeg = headingDeg,
            mode = mode, modeConfidence = if (demoActive) 1.0 else classifier.probability(mode),
            stepCount = stepCount, pathLength = pathLength, strideK = steps.weinbergK,
            calibrating = calibrating, calibrationDistance = calLength,
            visionConfidence = visionConf, visionActive = visionActive, visionCalibrated = visionCalibrated,
            visionStill = lastAccT - visionStillT < 300_000_000L,
            visionVelE = visionVelE, visionVelN = visionVelN, visionDepth = exp(logDepth),
            visionConsistency = visionConsistency,
            magReliability = orientation.magReliability,
            trajectory = cachedTraj, explored = cachedExplored, exploredArea = cachedExplored.size.toDouble(),
            guidance = g, demoActive = demoActive,
            demoTruth = if (demoActive) Point2D(demoE, demoN) else null
        )
    }

    // ============================== internals ==============================

    private fun runClassifier(t: Long) {
        classifier.update(MotionClassifier.Features(accStats.std, gyroStats.mean, steps.cadence(t)))
        if (!aligned && orientation.initialized && t - startNs > 2_000_000_000L) aligned = true
        if (demoActive) return
        // Nothing observed velocity recently (no steps, no vision) → gently damp it.
        val noSteps = t - lastStepT > 1_200_000_000L
        val noVision = t - lastVisionUpdateT > 500_000_000L
        if (noSteps && noVision && classifier.mode != MotionMode.STATIONARY) {
            position.updateVelocity(0.0, 0.0, if (classifier.mode == MotionMode.HANDLING) 0.15 else 0.4)
        }
    }

    private class PendingStep(val dE: Double, val dN: Double, val length: Double, val t: Long)

    /**
     * PDR update. Each confirmed step gives two measurements:
     *  - velocity  = stride / step interval along the heading
     *  - position  = position at previous step + stride vector (keeps distance exact, no corner lag)
     * Steps detected before the classifier confirmed walking are held briefly and credited later.
     */
    private fun applyStep(length: Double, interval: Double, bearingRad: Double, t: Long, confirmed: Boolean = true) {
        val dE = length * sin(bearingRad); val dN = length * cos(bearingRad)
        pending.removeAll { t - it.t > 2_500_000_000L }
        if (!confirmed) { pending.add(PendingStep(dE, dN, length, t)); return }

        var sumE = dE; var sumN = dN; var sumL = length; var n = 1
        for (p in pending) { sumE += p.dE; sumN += p.dN; sumL += p.length; n++ }
        pending.clear()

        val v = length / interval.coerceIn(0.3, 1.2)
        val vE = v * sin(bearingRad); val vN = v * cos(bearingRad)
        if (anchorT != 0L && t - anchorT < 2_500_000_000L) {
            position.updatePosition(anchorE + sumE, anchorN + sumN, 0.05 + 0.04 * n)
        } else if (n > 1) {
            position.shift(sumE - dE, sumN - dN)
        }
        position.updateVelocity(vE, vN, 0.2)
        anchorE = position.east; anchorN = position.north; anchorT = t

        lastStepT = t; lastStepVelE = vE; lastStepVelN = vN
        stepCount += n; pathLength += sumL
        if (calibrating) calLength += sumL
    }

    private fun recordTrajectory() {
        val last = trajectory.last()
        val e = position.east; val n = position.north
        if (hypot(e - last.x, n - last.y) > 0.05) {
            trajectory.add(Point2D(e, n))
            if (trajectory.size > 6000) { // decimate the oldest half
                val keep = ArrayList<Point2D>(4500)
                for (i in trajectory.indices) if (i >= 3000 || i % 2 == 0) keep.add(trajectory[i])
                trajectory.clear(); trajectory.addAll(keep)
            }
            markExplored(e, n)
            trajDirty = true
        }
    }

    /** Marks 1 m cells within ~2 m of the rescuer as explored (approximate sensing footprint). */
    private fun markExplored(e: Double, n: Double) {
        val cx = floor(e).toInt(); val cy = floor(n).toInt()
        for (i in -2..2) for (j in -2..2) {
            if (i * i + j * j > 5) continue
            explored.add(cellKey(cx + i, cy + j))
        }
    }

    private fun rotateCw(u: Double, v: Double, deg: Int): Pair<Double, Double> = when (((deg % 360) + 360) % 360) {
        90 -> -v to u
        180 -> -u to -v
        270 -> v to -u
        else -> u to v
    }

    companion object {
        const val DRIFT_FRACTION = 0.03
        fun cellKey(ix: Int, iy: Int): Long = (ix.toLong() shl 32) or (iy.toLong() and 0xFFFFFFFFL)
        fun cellX(key: Long): Int = (key shr 32).toInt()
        fun cellY(key: Long): Int = key.toInt()
    }
}
