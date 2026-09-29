package com.deadrecon.app.nav

import kotlin.math.abs

typealias Matrix = Array<DoubleArray>

/** Minimal dense-matrix helpers. Sizes here are tiny (≤ 6x6), so clarity beats speed. */
object Mat {
    fun zeros(r: Int, c: Int): Matrix = Array(r) { DoubleArray(c) }

    fun identity(n: Int, s: Double = 1.0): Matrix =
        Array(n) { i -> DoubleArray(n) { j -> if (i == j) s else 0.0 } }

    fun t(a: Matrix): Matrix = Array(a[0].size) { j -> DoubleArray(a.size) { i -> a[i][j] } }

    fun mul(a: Matrix, b: Matrix): Matrix {
        val n = a.size; val m = b[0].size; val k = b.size
        val r = zeros(n, m)
        for (i in 0 until n) for (p in 0 until k) {
            val aip = a[i][p]
            if (aip == 0.0) continue
            for (j in 0 until m) r[i][j] += aip * b[p][j]
        }
        return r
    }

    fun mulVec(a: Matrix, v: DoubleArray): DoubleArray =
        DoubleArray(a.size) { i -> var s = 0.0; for (j in v.indices) s += a[i][j] * v[j]; s }

    fun add(a: Matrix, b: Matrix): Matrix =
        Array(a.size) { i -> DoubleArray(a[0].size) { j -> a[i][j] + b[i][j] } }

    /** Gauss-Jordan inverse with partial pivoting. Returns null if singular. */
    fun inv(a: Matrix): Matrix? {
        val n = a.size
        val m = Array(n) { i -> DoubleArray(2 * n) { j -> if (j < n) a[i][j] else if (j - n == i) 1.0 else 0.0 } }
        for (col in 0 until n) {
            var piv = col
            for (r in col + 1 until n) if (abs(m[r][col]) > abs(m[piv][col])) piv = r
            if (abs(m[piv][col]) < 1e-14) return null
            if (piv != col) { val tmp = m[piv]; m[piv] = m[col]; m[col] = tmp }
            val d = m[col][col]
            for (j in 0 until 2 * n) m[col][j] /= d
            for (r in 0 until n) if (r != col) {
                val f = m[r][col]
                if (f != 0.0) for (j in 0 until 2 * n) m[r][j] -= f * m[col][j]
            }
        }
        return Array(n) { i -> DoubleArray(n) { j -> m[i][j + n] } }
    }

    fun copyInto(src: Matrix, dst: Matrix) {
        for (i in src.indices) src[i].copyInto(dst[i])
    }

    fun symmetrize(a: Matrix) {
        for (i in a.indices) for (j in i + 1 until a.size) {
            val v = 0.5 * (a[i][j] + a[j][i]); a[i][j] = v; a[j][i] = v
        }
    }
}

/** Generic Kalman measurement update (Joseph form) with optional chi-square innovation gate. */
object Ekf {
    data class Outcome(val accepted: Boolean, val nis: Double)

    fun update(
        x: DoubleArray,
        P: Matrix,
        y: DoubleArray,
        H: Matrix,
        R: Matrix,
        gate: Double = Double.POSITIVE_INFINITY
    ): Outcome {
        val Ht = Mat.t(H)
        val PHt = Mat.mul(P, Ht)
        val S = Mat.add(Mat.mul(H, PHt), R)
        val Si = Mat.inv(S) ?: return Outcome(false, Double.NaN)
        val Siy = Mat.mulVec(Si, y)
        var nis = 0.0
        for (i in y.indices) nis += y[i] * Siy[i]
        if (nis > gate) return Outcome(false, nis)

        val K = Mat.mul(PHt, Si)
        val dx = Mat.mulVec(K, y)
        for (i in x.indices) x[i] += dx[i]

        val n = x.size
        val KH = Mat.mul(K, H)
        val IKH = Array(n) { i -> DoubleArray(n) { j -> (if (i == j) 1.0 else 0.0) - KH[i][j] } }
        val newP = Mat.add(Mat.mul(Mat.mul(IKH, P), Mat.t(IKH)), Mat.mul(Mat.mul(K, R), Mat.t(K)))
        Mat.symmetrize(newP)
        Mat.copyInto(newP, P)
        return Outcome(true, nis)
    }
}

/** Fixed-size rolling window with O(1) mean / variance. */
class RollingStats(private val capacity: Int) {
    private val data = DoubleArray(capacity)
    private var head = 0
    var count = 0; private set
    private var sum = 0.0
    private var sumSq = 0.0

    fun add(v: Double) {
        if (count == capacity) { val old = data[head]; sum -= old; sumSq -= old * old } else count++
        data[head] = v; sum += v; sumSq += v * v
        head = (head + 1) % capacity
    }

    val mean: Double get() = if (count == 0) 0.0 else sum / count
    val variance: Double get() = if (count < 2) 0.0 else maxOf(0.0, sumSq / count - mean * mean)
    val std: Double get() = kotlin.math.sqrt(variance)
    val full: Boolean get() = count == capacity

    fun clear() { head = 0; count = 0; sum = 0.0; sumSq = 0.0 }
}
