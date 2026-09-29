package com.deadrecon.app.map

import com.deadrecon.app.nav.Point2D

/**
 * Dummy demo map: a fictional underground parking level, in metres relative to the start
 * point (entrance = 0,0; X = East, Y = North). Purely illustrative — it is drawn under the
 * live trajectory so the pointer has something to trace. Replace with a real floor plan or
 * a LiDAR-built occupancy grid later.
 */
object DemoMap {

    enum class Kind { FLOOR, ROOM, SAFE, HAZARD, TARGET }

    data class Area(val x0: Double, val y0: Double, val x1: Double, val y1: Double, val kind: Kind, val label: String? = null)
    data class Marker(val x: Double, val y: Double, val label: String, val kind: Kind)

    const val TITLE = "DEMO · Basement B2 (fictional)"

    val areas = listOf(
        Area(-4.0, -4.0, 4.0, 4.0, Kind.SAFE, "ENTRY / SAFE ZONE"),
        Area(-1.6, 3.5, 1.6, 13.6, Kind.FLOOR, "CORRIDOR A"),
        Area(-9.5, 8.5, -1.2, 16.0, Kind.ROOM, "UTILITY"),
        Area(-1.6, 10.4, 15.6, 13.6, Kind.FLOOR),
        Area(12.4, 10.4, 15.6, 29.6, Kind.FLOOR, "CORRIDOR B"),
        Area(16.2, 17.0, 25.0, 25.0, Kind.HAZARD, "COLLAPSED"),
        Area(12.4, 26.4, 31.6, 29.6, Kind.FLOOR, "CORRIDOR C"),
        Area(26.5, 29.2, 34.5, 37.0, Kind.ROOM, "STAIRWELL A"),
        Area(28.4, 14.5, 31.6, 29.6, Kind.FLOOR),
        Area(27.0, 6.5, 42.0, 17.5, Kind.ROOM, "PLANT ROOM"),
    )

    val markers = listOf(
        Marker(37.0, 12.0, "VICTIM REPORTED", Kind.TARGET),
        Marker(20.6, 21.0, "NO ENTRY", Kind.HAZARD),
    )

    /** Route the demo walker follows (stays inside the corridors). */
    val demoRoute = listOf(
        Point2D(0.0, 0.0), Point2D(0.0, 12.0), Point2D(14.0, 12.0), Point2D(14.0, 28.0),
        Point2D(30.0, 28.0), Point2D(30.0, 15.5), Point2D(37.0, 12.0)
    )
}
