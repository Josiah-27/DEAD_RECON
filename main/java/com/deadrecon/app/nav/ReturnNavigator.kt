package com.deadrecon.app.nav

import kotlin.math.*

data class Point2D(val x: Double, val y: Double)

data class ReturnGuidance(
    val active: Boolean,
    val targetE: Double = 0.0,
    val targetN: Double = 0.0,
    val distanceToWaypoint: Double = 0.0,
    val bearingToWaypointDeg: Double = 0.0,
    val turnDeg: Double = 0.0,            // + = turn right, − = turn left
    val straightLineToStart: Double = 0.0,
    val pathRemaining: Double = 0.0,
    val arrived: Boolean = false
)

/**
 * Breadcrumb retrace: walks the recorded outbound path backwards, always pointing at a
 * waypoint ~2.5 m ahead on the reversed route. Loops in the path are short-cut automatically.
 */
class ReturnNavigator {
    private var path: List<Point2D> = emptyList()
    private var cumulative = DoubleArray(0)
    private var idx = 0
    var active = false; private set

    fun start(outbound: List<Point2D>) {
        path = if (outbound.isEmpty()) listOf(Point2D(0.0, 0.0)) else outbound
        cumulative = DoubleArray(path.size)
        for (i in 1 until path.size) cumulative[i] = cumulative[i - 1] + dist(path[i - 1], path[i])
        idx = path.lastIndex
        active = true
    }

    fun stop() { active = false }

    fun target(): Point2D? = if (active) path[idx] else null

    fun update(e: Double, n: Double, headingDeg: Double): ReturnGuidance {
        if (!active) return ReturnGuidance(false)
        val here = Point2D(e, n)

        // Short-cut: jump to the earliest nearby breadcrumb (handles loops / re-crossings).
        var best = idx
        for (k in 0 until idx) if (dist(here, path[k]) < 1.5) { best = k; break }
        idx = best
        while (idx > 0 && dist(here, path[idx]) < 2.5) idx--

        val t = path[idx]
        val d = dist(here, t)
        val bearing = (Math.toDegrees(atan2(t.x - e, t.y - n)) + 360) % 360
        var turn = bearing - headingDeg
        while (turn > 180) turn -= 360
        while (turn < -180) turn += 360
        val toStart = hypot(e, n)
        return ReturnGuidance(
            active = true, targetE = t.x, targetN = t.y,
            distanceToWaypoint = d, bearingToWaypointDeg = bearing, turnDeg = turn,
            straightLineToStart = toStart,
            pathRemaining = d + cumulative[idx],
            arrived = toStart < 1.5
        )
    }

    private fun dist(a: Point2D, b: Point2D) = hypot(a.x - b.x, a.y - b.y)
}
