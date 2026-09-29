package com.deadrecon.app.vision

import java.nio.ByteBuffer
import kotlin.math.*

/**
 * Sparse pyramidal Lucas-Kanade visual-motion engine.
 *
 *  1. Luma frame → 80×60 box-averaged grid (+ one half-resolution pyramid level)
 *  2. 24 feature windows tracked coarse-to-fine with iterative LK
 *  3. Shi-Tomasi minimum eigenvalue rejects texture-less windows (smoke, blank walls)
 *  4. Forward-backward consistency check rejects occlusions / moving debris
 *  5. Robust similarity-motion fit  flow = t + s·d + r·d⊥  (translation, expansion, roll)
 *     with outlier rejection — expansion is what reveals *forward* motion when the camera
 *     looks where the rescuer walks.
 *
 * Output is image-space motion per frame; turning it into metres per second (rotation
 * compensation, depth scale) happens in the navigation engine, which knows the gyro rates.
 */
class OpticalFlowEngine(val gridW: Int = 80, val gridH: Int = 60, hfovDeg: Double = 66.0) {

    /** Focal length expressed in grid pixels. */
    val focalPx: Double = (gridW / 2.0) / tan(Math.toRadians(hfovDeg / 2.0))

    data class Result(
        val valid: Boolean,
        val tx: Double = 0.0,          // px / frame, image axes
        val ty: Double = 0.0,
        val expansion: Double = 0.0,   // fractional scale change / frame (+ = approaching scene)
        val roll: Double = 0.0,        // rad / frame
        val confidence: Double = 0.0,  // 0..1
        val inliers: Int = 0,
        val tracked: Int = 0,
        val texture: Double = 0.0,
        val brightness: Double = 0.0
    )

    private var prev0: FloatArray? = null
    private var prev1: FloatArray? = null
    private val w1 = gridW / 2
    private val h1 = gridH / 2

    private val centers: List<Pair<Int, Int>> = buildList {
        val cols = 6; val rows = 4; val margin = 11
        for (r in 0 until rows) for (c in 0 until cols) {
            val x = margin + (gridW - 1 - 2 * margin) * c / (cols - 1)
            val y = margin + (gridH - 1 - 2 * margin) * r / (rows - 1)
            add(x to y)
        }
    }

    fun reset() { prev0 = null; prev1 = null }

    /** Box-averaged (2×2 taps per cell) luma downsample straight from the Y plane. */
    fun downsample(buffer: ByteBuffer, srcW: Int, srcH: Int, rowStride: Int, pixelStride: Int = 1): FloatArray {
        val out = FloatArray(gridW * gridH)
        val cw = srcW.toDouble() / gridW
        val ch = srcH.toDouble() / gridH
        val limit = buffer.limit()
        for (gy in 0 until gridH) {
            val y0 = (gy * ch + ch * 0.25).toInt().coerceIn(0, srcH - 1)
            val y1 = (gy * ch + ch * 0.75).toInt().coerceIn(0, srcH - 1)
            for (gx in 0 until gridW) {
                val x0 = (gx * cw + cw * 0.25).toInt().coerceIn(0, srcW - 1)
                val x1 = (gx * cw + cw * 0.75).toInt().coerceIn(0, srcW - 1)
                var s = 0
                for (yy in intArrayOf(y0, y1)) for (xx in intArrayOf(x0, x1)) {
                    val idx = yy * rowStride + xx * pixelStride
                    if (idx < limit) s += buffer.get(idx).toInt() and 0xFF
                }
                out[gy * gridW + gx] = s / (4f * 255f)
            }
        }
        return out
    }

    fun process(cur0: FloatArray): Result {
        val cur1 = halve(cur0, gridW, gridH)
        val p0 = prev0; val p1 = prev1
        prev0 = cur0; prev1 = cur1
        val brightness = cur0.average()
        if (p0 == null || p1 == null) return Result(false, brightness = brightness)

        val px = ArrayList<Double>(); val py = ArrayList<Double>()
        val fx = ArrayList<Double>(); val fy = ArrayList<Double>()
        val tex = ArrayList<Double>()

        for ((cx, cy) in centers) {
            val coarse = lk(p1, cur1, w1, h1, cx / 2, cy / 2, 3, 0.0, 0.0) ?: continue
            val fine = lk(p0, cur0, gridW, gridH, cx, cy, 5, 2 * coarse[0], 2 * coarse[1]) ?: continue
            if (fine[2] < MIN_EIGEN) continue
            val back = lk(cur0, p0, gridW, gridH, cx, cy, 5, -fine[0], -fine[1]) ?: continue
            if (hypot(fine[0] + back[0], fine[1] + back[1]) > 0.6) continue
            px += cx - gridW / 2.0; py += cy - gridH / 2.0
            fx += fine[0]; fy += fine[1]; tex += fine[2]
        }
        val tracked = px.size
        if (tracked < 5) return Result(false, tracked = tracked, brightness = brightness)

        var inl = BooleanArray(tracked) { true }
        var model = fit(px, py, fx, fy, inl) ?: return Result(false, tracked = tracked, brightness = brightness)
        val resid = DoubleArray(tracked) { residual(model, px[it], py[it], fx[it], fy[it]) }
        val med = resid.sorted()[tracked / 2]
        val thr = max(0.35, 2.5 * med)
        inl = BooleanArray(tracked) { resid[it] <= thr }
        val nIn = inl.count { it }
        if (nIn < 5) return Result(false, tracked = tracked, brightness = brightness)
        model = fit(px, py, fx, fy, inl) ?: return Result(false, tracked = tracked, brightness = brightness)

        var rss = 0.0
        for (i in 0 until tracked) if (inl[i]) rss += residual(model, px[i], py[i], fx[i], fy[i]).pow(2)
        val rms = sqrt(rss / nIn)
        val texMed = tex.sorted()[tex.size / 2]

        val inlierScore = (nIn.toDouble() / centers.size).pow(0.7)
        val textureScore = (texMed / 0.0015).coerceIn(0.0, 1.0)
        val residualScore = 1.0 / (1.0 + (rms / 0.3).pow(2))
        val exposureScore = if (brightness in 0.06..0.95) 1.0 else 0.2
        val conf = (inlierScore * textureScore * residualScore * exposureScore).coerceIn(0.0, 1.0)

        return Result(true, model[0], model[1], model[2], model[3], conf, nIn, tracked, texMed, brightness)
    }

    // ---- internals -------------------------------------------------------------------------

    /** Least-squares fit of [tx, ty, s, r]. */
    private fun fit(px: List<Double>, py: List<Double>, fx: List<Double>, fy: List<Double>, use: BooleanArray): DoubleArray? {
        val ata = Array(4) { DoubleArray(4) }
        val atb = DoubleArray(4)
        for (i in px.indices) {
            if (!use[i]) continue
            val rx = doubleArrayOf(1.0, 0.0, px[i], -py[i])
            val ry = doubleArrayOf(0.0, 1.0, py[i], px[i])
            for (a in 0..3) {
                atb[a] += rx[a] * fx[i] + ry[a] * fy[i]
                for (b in 0..3) ata[a][b] += rx[a] * rx[b] + ry[a] * ry[b]
            }
        }
        for (a in 0..3) ata[a][a] += 1e-9
        val inv = com.deadrecon.app.nav.Mat.inv(ata) ?: return null
        return com.deadrecon.app.nav.Mat.mulVec(inv, atb)
    }

    private fun residual(m: DoubleArray, x: Double, y: Double, fx: Double, fy: Double): Double {
        val ex = m[0] + m[2] * x - m[3] * y
        val ey = m[1] + m[2] * y + m[3] * x
        return hypot(fx - ex, fy - ey)
    }

    /**
     * Iterative Lucas-Kanade for one window centred at integer (cx, cy) in I, searched in J.
     * Returns [dx, dy, λmin] or null if the window leaves the image / is degenerate.
     */
    private fun lk(I: FloatArray, J: FloatArray, w: Int, h: Int, cx: Int, cy: Int, half: Int,
                   gx0: Double, gy0: Double, iters: Int = 6): DoubleArray? {
        if (cx - half < 1 || cy - half < 1 || cx + half > w - 2 || cy + half > h - 2) return null
        val n = (2 * half + 1) * (2 * half + 1)
        val ix = DoubleArray(n); val iy = DoubleArray(n); val iv = DoubleArray(n)
        val xs = IntArray(n); val ys = IntArray(n)
        var gxx = 0.0; var gxy = 0.0; var gyy = 0.0
        var k = 0
        for (dy in -half..half) for (dx in -half..half) {
            val x = cx + dx; val y = cy + dy; val i = y * w + x
            ix[k] = (I[i + 1] - I[i - 1]) * 0.5
            iy[k] = (I[i + w] - I[i - w]) * 0.5
            iv[k] = I[i].toDouble()
            xs[k] = x; ys[k] = y
            gxx += ix[k] * ix[k]; gxy += ix[k] * iy[k]; gyy += iy[k] * iy[k]
            k++
        }
        val det = gxx * gyy - gxy * gxy
        if (det < 1e-12) return null
        val lambdaMin = ((gxx + gyy) - sqrt((gxx - gyy).pow(2) + 4 * gxy * gxy)) / 2.0 / n

        var dx = gx0; var dy = gy0
        repeat(iters) {
            var bx = 0.0; var by = 0.0
            for (j in 0 until n) {
                val qx = xs[j] + dx; val qy = ys[j] + dy
                if (qx < 0 || qy < 0 || qx > w - 1.001 || qy > h - 1.001) return null
                val diff = iv[j] - bilinear(J, w, qx, qy)
                bx += diff * ix[j]; by += diff * iy[j]
            }
            val ddx = (gyy * bx - gxy * by) / det
            val ddy = (gxx * by - gxy * bx) / det
            dx += ddx; dy += ddy
            if (abs(dx) > half * 2.5 || abs(dy) > half * 2.5) return null
            if (ddx * ddx + ddy * ddy < 1e-4) return doubleArrayOf(dx, dy, lambdaMin)
        }
        return doubleArrayOf(dx, dy, lambdaMin)
    }

    private fun bilinear(img: FloatArray, w: Int, x: Double, y: Double): Double {
        val x0 = x.toInt(); val y0 = y.toInt()
        val ax = x - x0; val ay = y - y0
        val i = y0 * w + x0
        val a = img[i]; val b = img[i + 1]; val c = img[i + w]; val d = img[i + w + 1]
        return (a * (1 - ax) + b * ax) * (1 - ay) + (c * (1 - ax) + d * ax) * ay
    }

    private fun halve(src: FloatArray, w: Int, h: Int): FloatArray {
        val hw = w / 2; val hh = h / 2
        return FloatArray(hw * hh) { i ->
            val x = (i % hw) * 2; val y = (i / hw) * 2
            (src[y * w + x] + src[y * w + x + 1] + src[(y + 1) * w + x] + src[(y + 1) * w + x + 1]) * 0.25f
        }
    }

    companion object {
        private const val MIN_EIGEN = 1.5e-4
    }
}
