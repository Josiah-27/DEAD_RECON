package com.deadrecon.app.nav

import kotlin.math.*

/**
 * Quaternion orientation EKF.
 *
 * Frames: body = Android sensor frame (X right, Y up the screen, Z out of the screen).
 * World = ENU (X = East, Y = North, Z = Up). q = [w, x, y, z] rotates body → world.
 *
 *  - Gyro:          prediction (exact quaternion exponential, bias-corrected)
 *  - Accelerometer: gravity-direction update (adaptive noise, gated during dynamics)
 *  - Magnetometer:  scalar yaw update with disturbance detection (steel/rebar/machinery)
 */
class OrientationEkf {
    val q = doubleArrayOf(1.0, 0.0, 0.0, 0.0)
    private val P = Mat.identity(4, 1e-2)
    val gyroBias = DoubleArray(3)

    var initialized = false; private set
    /** 0..1 — how much the current magnetic field looks like the undisturbed reference. */
    var magReliability = 0.0; private set
    var lastMagAccepted = false; private set

    private var magRefNorm = 0.0
    private var magRefDip = 0.0
    private var magRefCount = 0

    private val gyroNoise = 4e-5
    private val gravity = 9.80665

    fun reset() {
        q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0
        Mat.copyInto(Mat.identity(4, 1e-2), P)
        initialized = false
        magRefCount = 0; magReliability = 0.0
    }

    /** TRIAD alignment from one accelerometer (+ optional magnetometer) sample. */
    fun initialize(acc: DoubleArray, mag: DoubleArray?) {
        val up = normalize(acc) ?: return
        var east = mag?.let { normalize(cross(it, up)) }
        if (east == null) {
            // No compass: define "north" as the phone's top edge projected on the horizontal.
            val yb = doubleArrayOf(0.0, 1.0, 0.0)
            val d = dot(yb, up)
            val n0 = normalize(doubleArrayOf(yb[0] - d * up[0], yb[1] - d * up[1], yb[2] - d * up[2]))
                ?: doubleArrayOf(0.0, 0.0, -1.0)
            east = normalize(cross(n0, up)) ?: return
        }
        val north = cross(up, east)
        // Rows of body→world rotation are the world axes expressed in body coordinates.
        setFromMatrix(arrayOf(east, north, up))
        Mat.copyInto(Mat.identity(4, 1e-3), P)
        initialized = true
    }

    fun predict(wxRaw: Double, wyRaw: Double, wzRaw: Double, dt: Double) {
        if (dt <= 0.0 || dt > 0.2) return
        val wx = wxRaw - gyroBias[0]; val wy = wyRaw - gyroBias[1]; val wz = wzRaw - gyroBias[2]
        val wn = sqrt(wx * wx + wy * wy + wz * wz)
        val half = 0.5 * wn * dt
        val dq = if (wn > 1e-9) {
            val s = sin(half) / wn
            doubleArrayOf(cos(half), wx * s, wy * s, wz * s)
        } else doubleArrayOf(1.0, 0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt)

        // q_new = q ⊗ dq  ==  F · q
        val F = arrayOf(
            doubleArrayOf(dq[0], -dq[1], -dq[2], -dq[3]),
            doubleArrayOf(dq[1], dq[0], dq[3], -dq[2]),
            doubleArrayOf(dq[2], -dq[3], dq[0], dq[1]),
            doubleArrayOf(dq[3], dq[2], -dq[1], dq[0])
        )
        val qn = Mat.mulVec(F, q)
        qn.copyInto(q); normalizeQ()

        val FP = Mat.mul(Mat.mul(F, P), Mat.t(F))
        for (i in 0..3) FP[i][i] += gyroNoise * dt
        Mat.copyInto(FP, P)
    }

    /** Gravity-direction update. Returns true when used. */
    fun updateAccel(ax: Double, ay: Double, az: Double): Boolean {
        val n = sqrt(ax * ax + ay * ay + az * az)
        val dev = abs(n - gravity)
        if (n < 1e-3 || dev > 1.5) return false
        val z = doubleArrayOf(ax / n, ay / n, az / n)
        val (w, x, y, zq) = q.toList()
        val h = doubleArrayOf(
            2.0 * (x * zq - w * y),
            2.0 * (w * x + y * zq),
            w * w - x * x - y * y + zq * zq
        )
        val H = arrayOf(
            doubleArrayOf(-2.0 * y, 2.0 * zq, -2.0 * w, 2.0 * x),
            doubleArrayOf(2.0 * x, 2.0 * w, 2.0 * zq, 2.0 * y),
            doubleArrayOf(2.0 * w, -2.0 * x, -2.0 * y, 2.0 * zq)
        )
        val r = 0.03 * (1.0 + 6.0 * dev).pow(2)
        val res = doubleArrayOf(z[0] - h[0], z[1] - h[1], z[2] - h[2])
        val out = Ekf.update(q, P, res, H, Mat.identity(3, r))
        normalizeQ()
        return out.accepted
    }

    /**
     * Yaw update from a tilt-compensated compass heading.
     * The field reference (magnitude + dip) is learnt at start-up; later readings that deviate
     * from it are down-weighted or rejected, which matters a lot inside steel-rich structures.
     */
    fun updateMag(mx: Double, my: Double, mz: Double, declinationRad: Double): Boolean {
        lastMagAccepted = false
        if (!initialized) return false
        val norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm < 1e-3) return false
        val m = doubleArrayOf(mx / norm, my / norm, mz / norm)
        val w = Mat.mulVec(rotationMatrix(q), m)
        val dip = atan2(-w[2], hypot(w[0], w[1]))

        if (magRefCount < 150) {
            magRefCount++
            magRefNorm += (norm - magRefNorm) / magRefCount
            magRefDip += (dip - magRefDip) / magRefCount
            magReliability = 0.7
        } else {
            val normDev = abs(norm - magRefNorm) / magRefNorm
            val dipDev = abs(dip - magRefDip)
            magReliability = (1.0 - normDev / 0.18 - dipDev / 0.30).coerceIn(0.0, 1.0)
            if (magReliability > 0.8) { // slow adaptation to genuine environment drift
                magRefNorm += 0.002 * (norm - magRefNorm)
                magRefDip += 0.002 * (dip - magRefDip)
            }
        }
        if (magReliability < 0.35) return false

        val delta = compassAngle(q, m)
        val res = doubleArrayOf(wrapPi(declinationRad - delta))
        val eps = 1e-6
        val H = arrayOf(DoubleArray(4) { i ->
            val qp = q.copyOf(); qp[i] += eps
            val nrm = sqrt(qp.sumOf { it * it }); for (k in 0..3) qp[k] /= nrm
            wrapPi(compassAngle(qp, m) - delta) / eps
        })
        val sigma = 0.15 / sqrt(magReliability)
        val out = Ekf.update(q, P, res, H, arrayOf(doubleArrayOf(sigma * sigma)), gate = 10.83)
        normalizeQ()
        lastMagAccepted = out.accepted
        return out.accepted
    }

    /** Called only while the device is confidently stationary. */
    fun learnGyroBias(wx: Double, wy: Double, wz: Double, alpha: Double = 0.01) {
        gyroBias[0] += alpha * (wx - gyroBias[0])
        gyroBias[1] += alpha * (wy - gyroBias[1])
        gyroBias[2] += alpha * (wz - gyroBias[2])
    }

    fun rotationMatrix(): Matrix = rotationMatrix(q)

    fun bodyToWorld(v: DoubleArray): DoubleArray = Mat.mulVec(rotationMatrix(q), v)

    /**
     * Direction of travel as a bearing (radians, clockwise from north).
     * Blends the phone's top edge and the camera axis so it works both flat and held upright.
     */
    fun travelBearing(): Double {
        val r = rotationMatrix(q)
        val topE = r[0][1]; val topN = r[1][1]       // body +Y in world
        val camE = -r[0][2]; val camN = -r[1][2]     // body -Z (back camera) in world
        return atan2(topE + camE, topN + camN)
    }

    private fun setFromMatrix(m: Matrix) {
        val tr = m[0][0] + m[1][1] + m[2][2]
        if (tr > 0) {
            val s = sqrt(tr + 1.0) * 2
            q[0] = 0.25 * s; q[1] = (m[2][1] - m[1][2]) / s; q[2] = (m[0][2] - m[2][0]) / s; q[3] = (m[1][0] - m[0][1]) / s
        } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
            val s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
            q[0] = (m[2][1] - m[1][2]) / s; q[1] = 0.25 * s; q[2] = (m[0][1] + m[1][0]) / s; q[3] = (m[0][2] + m[2][0]) / s
        } else if (m[1][1] > m[2][2]) {
            val s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
            q[0] = (m[0][2] - m[2][0]) / s; q[1] = (m[0][1] + m[1][0]) / s; q[2] = 0.25 * s; q[3] = (m[1][2] + m[2][1]) / s
        } else {
            val s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
            q[0] = (m[1][0] - m[0][1]) / s; q[1] = (m[0][2] + m[2][0]) / s; q[2] = (m[1][2] + m[2][1]) / s; q[3] = 0.25 * s
        }
        normalizeQ()
    }

    private fun normalizeQ() {
        val n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if (n < 1e-12 || n.isNaN()) { q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0; return }
        for (i in 0..3) q[i] /= n
    }

    companion object {
        fun rotationMatrix(q: DoubleArray): Matrix {
            val (w, x, y, z) = q.toList()
            return arrayOf(
                doubleArrayOf(1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                doubleArrayOf(2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                doubleArrayOf(2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))
            )
        }

        /** Bearing of the horizontal magnetic field in the current world frame. */
        private fun compassAngle(q: DoubleArray, mBody: DoubleArray): Double {
            val w = Mat.mulVec(rotationMatrix(q), mBody)
            return atan2(w[0], w[1])
        }

        fun wrapPi(a: Double): Double {
            var r = a
            while (r > PI) r -= 2 * PI
            while (r < -PI) r += 2 * PI
            return r
        }

        fun cross(a: DoubleArray, b: DoubleArray) = doubleArrayOf(
            a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]
        )

        fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        fun normalize(v: DoubleArray): DoubleArray? {
            val n = sqrt(dot(v, v))
            return if (n < 1e-9) null else doubleArrayOf(v[0] / n, v[1] / n, v[2] / n)
        }
    }
}
