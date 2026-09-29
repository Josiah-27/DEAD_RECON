package com.deadrecon.app.ai

import kotlin.math.exp
import kotlin.math.ln

enum class MotionMode(val label: String) {
    STATIONARY("STATIONARY"),
    WALKING("WALKING"),
    RUNNING("RUNNING"),
    CRAWLING("CRAWL / IRREGULAR"),
    HANDLING("DEVICE HANDLING")
}

/**
 * On-device motion-mode recogniser: a Hidden Markov Model with Gaussian emission models.
 *
 * Features (2-second window):
 *   accStd   – std-dev of |accel|           (body-impact energy)
 *   gyroMean – mean |ω| after bias removal   (rotation energy)
 *   cadence  – detected steps per second     (gait periodicity)
 *
 * The forward algorithm smooths decisions over time, so one noisy window cannot flip the mode.
 * Emission parameters below are hand-set priors from typical smartphone gait data; they can be
 * replaced by per-class means/std-devs fitted from logged sessions (see [setEmission]) without
 * touching the rest of the pipeline.
 *
 * The mode drives the navigation filter: ZUPT + gyro-bias learning when stationary, step
 * updates while walking/running, vision-led velocity when crawling, and motion damping when the
 * rescuer is only turning the phone in their hand.
 */
class MotionClassifier {

    data class Features(val accStd: Double, val gyroMean: Double, val cadence: Double)

    private class Gaussian(val mean: Double, val std: Double)

    private val modes = MotionMode.values()
    private val emission: Array<Array<Gaussian>> = arrayOf(
        //                accStd               gyroMean              cadence
        arrayOf(Gaussian(0.04, 0.05), Gaussian(0.03, 0.04), Gaussian(0.0, 0.15)), // STATIONARY
        arrayOf(Gaussian(1.60, 0.90), Gaussian(0.45, 0.35), Gaussian(1.8, 0.40)), // WALKING
        arrayOf(Gaussian(5.00, 2.50), Gaussian(1.50, 1.00), Gaussian(2.8, 0.45)), // RUNNING
        arrayOf(Gaussian(0.70, 0.45), Gaussian(0.40, 0.30), Gaussian(0.1, 0.30)), // CRAWLING
        arrayOf(Gaussian(0.35, 0.30), Gaussian(1.40, 0.80), Gaussian(0.0, 0.25))  // HANDLING
    )

    private val stay = 0.90
    val posterior = DoubleArray(modes.size) { 1.0 / modes.size }
    var mode = MotionMode.STATIONARY; private set

    fun reset() {
        posterior.fill(1.0 / modes.size); mode = MotionMode.STATIONARY
    }

    fun setEmission(mode: MotionMode, feature: Int, mean: Double, std: Double) {
        emission[mode.ordinal][feature] = Gaussian(mean, std.coerceAtLeast(1e-3))
    }

    fun update(f: Features): MotionMode {
        val obs = doubleArrayOf(f.accStd, f.gyroMean, f.cadence)
        val ll = DoubleArray(modes.size) { m ->
            var s = 0.0
            for (k in obs.indices) {
                val g = emission[m][k]
                val z = (obs[k] - g.mean) / g.std
                s += -0.5 * z * z - ln(g.std)
            }
            s
        }
        val maxLl = ll.max()
        val switchP = (1 - stay) / (modes.size - 1)
        val prior = DoubleArray(modes.size) { i ->
            var p = 0.0
            for (j in modes.indices) p += posterior[j] * (if (i == j) stay else switchP)
            p
        }
        var total = 0.0
        for (i in modes.indices) { posterior[i] = prior[i] * exp(ll[i] - maxLl) + 1e-6; total += posterior[i] }
        for (i in modes.indices) posterior[i] /= total

        val best = posterior.indices.maxBy { posterior[it] }
        if (best != mode.ordinal && posterior[best] > 0.55) mode = modes[best]
        return mode
    }

    fun probability(m: MotionMode) = posterior[m.ordinal]
}
