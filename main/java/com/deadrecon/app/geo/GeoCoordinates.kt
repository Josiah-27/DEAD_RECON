package com.deadrecon.app.geo

import kotlin.math.*

/**
 * Geographic layer — deliberately independent of the sensor-fusion code.
 * It only ever receives local East/North displacements (metres) from the start point.
 */
data class LatLon(val latitude: Double, val longitude: Double) {
    init {
        require(latitude in -90.0..90.0) { "Latitude must be between -90 and 90" }
        require(longitude in -180.0..180.0) { "Longitude must be between -180 and 180" }
    }
}

/** WGS-84 ellipsoid constants. */
object Wgs84 {
    const val A = 6_378_137.0
    const val F = 1.0 / 298.257223563
    const val E2 = F * (2 - F)
    const val MEAN_RADIUS = 6_371_008.8
}

/**
 * Local tangent plane anchored at the user-entered start coordinate.
 *
 *   Δlat = north / M(φ)          Δlon = east / (N(φ) · cos φ)
 *
 * M and N are the WGS-84 meridional and prime-vertical radii of curvature. This is the
 * "displacement / Earth radius" formula with the ellipsoid's local radii instead of one sphere
 * radius (≈0.3 % more accurate), evaluated at the mid-latitude of the move.
 */
class LocalTangentPlane(val origin: LatLon) {

    fun toGeo(east: Double, north: Double): LatLon {
        val phi0 = Math.toRadians(origin.latitude)
        val dLat = north / meridionalRadius(phi0)
        val phiMid = phi0 + dLat / 2
        val cosMid = cos(phiMid).coerceAtLeast(1e-6)
        val dLon = east / (primeVerticalRadius(phiMid) * cosMid)
        val lat = (origin.latitude + Math.toDegrees(dLat)).coerceIn(-90.0, 90.0)
        return LatLon(lat, normalizeLon(origin.longitude + Math.toDegrees(dLon)))
    }

    /** Inverse: geographic point → (east, north) metres from the origin. */
    fun toLocal(p: LatLon): Pair<Double, Double> {
        val dLat = Math.toRadians(p.latitude - origin.latitude)
        val dLon = Math.toRadians(normalizeLon(p.longitude - origin.longitude))
        val phiMid = Math.toRadians(origin.latitude) + dLat / 2
        val north = dLat * meridionalRadius(Math.toRadians(origin.latitude))
        val east = dLon * primeVerticalRadius(phiMid) * cos(phiMid)
        return east to north
    }

    companion object {
        fun meridionalRadius(phi: Double): Double {
            val s = sin(phi)
            return Wgs84.A * (1 - Wgs84.E2) / (1 - Wgs84.E2 * s * s).pow(1.5)
        }

        fun primeVerticalRadius(phi: Double): Double {
            val s = sin(phi)
            return Wgs84.A / sqrt(1 - Wgs84.E2 * s * s)
        }

        fun normalizeLon(lon: Double): Double {
            var l = lon
            while (l > 180.0) l -= 360.0
            while (l < -180.0) l += 360.0
            return l
        }
    }
}

object GeoMath {
    /** Great-circle distance in metres. */
    fun distance(a: LatLon, b: LatLon): Double {
        val p1 = Math.toRadians(a.latitude); val p2 = Math.toRadians(b.latitude)
        val dp = p2 - p1; val dl = Math.toRadians(b.longitude - a.longitude)
        val h = sin(dp / 2).pow(2) + cos(p1) * cos(p2) * sin(dl / 2).pow(2)
        return 2 * Wgs84.MEAN_RADIUS * asin(sqrt(h.coerceIn(0.0, 1.0)))
    }

    /** Initial bearing a → b, degrees clockwise from true north. */
    fun bearing(a: LatLon, b: LatLon): Double {
        val p1 = Math.toRadians(a.latitude); val p2 = Math.toRadians(b.latitude)
        val dl = Math.toRadians(b.longitude - a.longitude)
        val y = sin(dl) * cos(p2)
        val x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl)
        return (Math.toDegrees(atan2(y, x)) + 360.0) % 360.0
    }
}

object GeoFormat {
    fun lat(v: Double) = "%.6f° %s".format(abs(v), if (v >= 0) "N" else "S")
    fun lon(v: Double) = "%.6f° %s".format(abs(v), if (v >= 0) "E" else "W")

    fun dms(v: Double, isLat: Boolean): String {
        val hemi = if (isLat) (if (v >= 0) "N" else "S") else (if (v >= 0) "E" else "W")
        val a = abs(v)
        var d = a.toInt()
        var m = ((a - d) * 60).toInt()
        var s = (a - d - m / 60.0) * 3600
        if (s >= 59.995) { s = 0.0; m += 1 }
        if (m >= 60) { m = 0; d += 1 }
        return "%d°%02d'%05.2f\"%s".format(d, m, s, hemi)
    }

    /** Compass point for a bearing in degrees. */
    fun cardinal(deg: Double): String {
        val names = arrayOf("N", "NE", "E", "SE", "S", "SW", "W", "NW")
        return names[(((deg % 360) + 360) % 360 / 45.0 + 0.5).toInt() % 8]
    }

    /**
     * Parses "12.9716", "-77.5946", "12.9716 N", "77.5946W" or "12°58'17.8\"N".
     * Returns null when the text isn't a valid coordinate for that axis.
     */
    fun parse(text: String, isLat: Boolean): Double? {
        val t = text.trim().uppercase().replace(",", ".")
        if (t.isEmpty()) return null
        val neg = t.endsWith("S") || t.endsWith("W")
        val pos = t.endsWith("N") || t.endsWith("E")
        if (isLat && (t.endsWith("E") || t.endsWith("W"))) return null
        if (!isLat && (t.endsWith("N") || t.endsWith("S"))) return null
        val body = if (neg || pos) t.dropLast(1).trim() else t
        val nums = Regex("-?\\d+(\\.\\d+)?").findAll(body).map { it.value.toDouble() }.toList()
        if (nums.isEmpty() || nums.size > 3) return null
        val sign = if (nums[0] < 0 || neg) -1.0 else 1.0
        if ((neg || pos) && nums[0] < 0) return null
        var v = abs(nums[0])
        if (nums.size >= 2) { if (nums[1] >= 60) return null; v += nums[1] / 60.0 }
        if (nums.size == 3) { if (nums[2] >= 60) return null; v += nums[2] / 3600.0 }
        v *= sign
        val limit = if (isLat) 90.0 else 180.0
        return if (abs(v) <= limit) v else null
    }
}
