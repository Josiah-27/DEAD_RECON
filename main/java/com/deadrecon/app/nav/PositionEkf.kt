package com.deadrecon.app.nav

/**
 * 2D position / velocity EKF with horizontal accelerometer-bias states.
 *
 * State x = [pE, pN, vE, vN, bE, bN]  (metres, m/s, m/s²; world ENU frame)
 *
 * Measurements:
 *  - ZUPT                    v = 0          (tight)
 *  - Pedestrian dead-reckoning step velocity (stride length × heading / step interval)
 *  - Visual-odometry velocity (scale self-calibrated against PDR)
 *  - Weak velocity damping when motion is irregular and nothing else observes it
 */
class PositionEkf {
    val x = DoubleArray(6)
    val P = Mat.zeros(6, 6)
    private val biasWalk = 0.02  // m/s² / √s

    init { reset() }

    fun reset() {
        x.fill(0.0)
        Mat.copyInto(Mat.zeros(6, 6), P)
        val d = doubleArrayOf(1e-4, 1e-4, 1e-2, 1e-2, 0.05, 0.05)
        for (i in 0..5) P[i][i] = d[i]
    }

    /** Makes the current position the origin (keeps velocity, biases and their covariance). */
    fun resetPosition() {
        x[0] = 0.0; x[1] = 0.0
        for (i in 0..5) { P[0][i] = 0.0; P[i][0] = 0.0; P[1][i] = 0.0; P[i][1] = 0.0 }
        P[0][0] = 1e-4; P[1][1] = 1e-4
    }

    /**
     * @param useAccel false → constant-velocity model (used while walking: the bounce of a hand-held
     *                 phone is noise relative to the step measurements).
     */
    fun predict(aE: Double, aN: Double, dt: Double, accelSigma: Double, useAccel: Boolean) {
        if (dt <= 0.0 || dt > 0.25) return
        val uE = if (useAccel) aE - x[4] else 0.0
        val uN = if (useAccel) aN - x[5] else 0.0
        val h = 0.5 * dt * dt
        x[0] += x[2] * dt + h * uE
        x[1] += x[3] * dt + h * uN
        x[2] += uE * dt
        x[3] += uN * dt

        val F = Mat.identity(6)
        F[0][2] = dt; F[1][3] = dt
        if (useAccel) { F[0][4] = -h; F[2][4] = -dt; F[1][5] = -h; F[3][5] = -dt }

        val FP = Mat.mul(Mat.mul(F, P), Mat.t(F))
        val s2 = accelSigma * accelSigma
        for (a in 0..1) {
            val p = a; val v = a + 2
            FP[p][p] += 0.25 * dt * dt * dt * dt * s2
            FP[p][v] += 0.5 * dt * dt * dt * s2
            FP[v][p] += 0.5 * dt * dt * dt * s2
            FP[v][v] += dt * dt * s2
            FP[a + 4][a + 4] += biasWalk * biasWalk * dt
        }
        Mat.symmetrize(FP)
        Mat.copyInto(FP, P)
        x[4] = x[4].coerceIn(-1.5, 1.5); x[5] = x[5].coerceIn(-1.5, 1.5)
    }

    fun updateVelocity(vE: Double, vN: Double, sigma: Double, gate: Double = Double.POSITIVE_INFINITY): Boolean {
        val H = arrayOf(
            doubleArrayOf(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            doubleArrayOf(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        )
        val y = doubleArrayOf(vE - x[2], vN - x[3])
        return Ekf.update(x, P, y, H, Mat.identity(2, sigma * sigma), gate).accepted
    }

    fun updatePosition(e: Double, n: Double, sigma: Double): Boolean {
        val H = arrayOf(
            doubleArrayOf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            doubleArrayOf(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
        )
        val y = doubleArrayOf(e - x[0], n - x[1])
        return Ekf.update(x, P, y, H, Mat.identity(2, sigma * sigma)).accepted
    }

    /** Direct shift (used to credit steps that happened before walking was confirmed). */
    fun shift(dE: Double, dN: Double) { x[0] += dE; x[1] += dN }

    fun zupt(sigma: Double = 0.01) { updateVelocity(0.0, 0.0, sigma) }

    val east get() = x[0]
    val north get() = x[1]
    val velE get() = x[2]
    val velN get() = x[3]
}
