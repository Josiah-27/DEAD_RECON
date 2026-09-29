package com.deadrecon.app.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipRect
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.deadrecon.app.map.DemoMap
import com.deadrecon.app.nav.NavSnapshot
import com.deadrecon.app.nav.NavigationEngine
import kotlin.math.*

/**
 * Live map: fictional demo floor plan + explored cells + trajectory + heading pointer +
 * 2σ uncertainty ellipse + return line. Pinch to zoom, drag to pan, tap RE-CENTER to follow.
 */
@Composable
fun MapPanel(snap: NavSnapshot, showDemoMap: Boolean, modifier: Modifier = Modifier) {
    var ppm by remember { mutableStateOf(12f) }         // pixels per metre
    var follow by remember { mutableStateOf(true) }
    var viewE by remember { mutableStateOf(0.0) }
    var viewN by remember { mutableStateOf(0.0) }
    val latest by rememberUpdatedState(snap)
    val measurer = rememberTextMeasurer()

    val centerE = if (follow) snap.east else viewE
    val centerN = if (follow) snap.north else viewN

    Box(modifier) {
        Canvas(
            Modifier
                .fillMaxSize()
                .pointerInput(Unit) {
                    detectTransformGestures { _, pan, zoom, _ ->
                        ppm = (ppm * zoom).coerceIn(2.5f, 80f)
                        if (pan.getDistance() > 1f) {
                            if (follow) { viewE = latest.east; viewN = latest.north; follow = false }
                            viewE -= pan.x / ppm
                            viewN += pan.y / ppm
                        }
                    }
                }
        ) {
            drawMap(snap, showDemoMap, centerE, centerN, ppm, measurer)
        }

        Column(Modifier.align(Alignment.TopEnd).padding(6.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            MapButton("+") { ppm = (ppm * 1.4f).coerceAtMost(80f) }
            MapButton("−") { ppm = (ppm / 1.4f).coerceAtLeast(2.5f) }
            MapButton(if (follow) "◉" else "○") { follow = true }
        }
        Text(
            if (showDemoMap) DemoMap.TITLE else "LOCAL FRAME",
            color = Console.Dim, fontSize = 10.sp, fontFamily = FontFamily.Monospace,
            modifier = Modifier.align(Alignment.TopStart).padding(8.dp)
        )
    }
}

@Composable
private fun MapButton(label: String, onClick: () -> Unit) {
    Box(
        Modifier
            .size(34.dp)
            .background(Console.Panel.copy(alpha = 0.85f), RoundedCornerShape(8.dp))
            .border(1.dp, Console.Line, RoundedCornerShape(8.dp))
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center
    ) { Text(label, color = Console.Text, fontSize = 16.sp, fontWeight = FontWeight.Bold) }
}

private fun DrawScope.drawMap(
    snap: NavSnapshot, showDemoMap: Boolean, cE: Double, cN: Double, ppm: Float, measurer: TextMeasurer
) {
    fun sx(e: Double) = (size.width / 2f + ((e - cE) * ppm).toFloat())
    fun sy(n: Double) = (size.height / 2f - ((n - cN) * ppm).toFloat())
    fun pt(e: Double, n: Double) = Offset(sx(e), sy(n))

    drawRect(Console.MapBg)

    // 5 m grid
    val step = if (ppm < 5f) 10.0 else 5.0
    val eMin = cE - size.width / 2 / ppm; val eMax = cE + size.width / 2 / ppm
    val nMin = cN - size.height / 2 / ppm; val nMax = cN + size.height / 2 / ppm
    var g = floor(eMin / step) * step
    while (g <= eMax) { drawLine(Console.Grid, Offset(sx(g), 0f), Offset(sx(g), size.height), 1f); g += step }
    g = floor(nMin / step) * step
    while (g <= nMax) { drawLine(Console.Grid, Offset(0f, sy(g)), Offset(size.width, sy(g)), 1f); g += step }

    // demo floor plan: walls (inflated outline) first, then floors over them → open junctions
    if (showDemoMap) {
        val wall = 0.35
        for (a in DemoMap.areas) {
            drawRect(
                Console.Wall,
                topLeft = pt(a.x0 - wall, a.y1 + wall),
                size = Size(((a.x1 - a.x0 + 2 * wall) * ppm).toFloat(), ((a.y1 - a.y0 + 2 * wall) * ppm).toFloat())
            )
        }
        for (a in DemoMap.areas) {
            val tl = pt(a.x0, a.y1)
            val sz = Size(((a.x1 - a.x0) * ppm).toFloat(), ((a.y1 - a.y0) * ppm).toFloat())
            val fill = when (a.kind) {
                DemoMap.Kind.SAFE -> Console.SafeFloor
                DemoMap.Kind.HAZARD -> Console.HazardFloor
                DemoMap.Kind.ROOM -> Console.RoomFloor
                else -> Console.Floor
            }
            drawRect(fill, tl, sz)
            if (a.kind == DemoMap.Kind.HAZARD) {
                clipRect(tl.x, tl.y, tl.x + sz.width, tl.y + sz.height) {
                    var h = -sz.height
                    while (h < sz.width) {
                        drawLine(Console.Red.copy(alpha = 0.35f), Offset(tl.x + h, tl.y + sz.height), Offset(tl.x + h + sz.height, tl.y), 2f)
                        h += 12f
                    }
                }
            }
        }
        if (ppm >= 6f) for (a in DemoMap.areas) a.label?.let {
            drawText(measurer, it, pt(a.x0, a.y1) + Offset(4f, 3f),
                TextStyle(color = Console.MapLabel, fontSize = 8.sp, fontFamily = FontFamily.Monospace))
        }
        for (m in DemoMap.markers) {
            val p = pt(m.x, m.y)
            val c = if (m.kind == DemoMap.Kind.TARGET) Console.Amber else Console.Red
            drawCircle(c, 7f, p)
            drawCircle(c.copy(alpha = 0.25f), 16f, p)
            if (ppm >= 6f) drawText(measurer, m.label, p + Offset(10f, -6f),
                TextStyle(color = c, fontSize = 8.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace))
        }
    }

    // explored coverage (1 m cells)
    val cellPx = ppm
    for (k in snap.explored) {
        val x = NavigationEngine.cellX(k).toDouble(); val y = NavigationEngine.cellY(k).toDouble()
        drawRect(Console.Green.copy(alpha = 0.10f), pt(x, y + 1), Size(cellPx, cellPx))
    }

    // trajectory
    val traj = snap.trajectory
    if (traj.size > 1) {
        val path = Path()
        path.moveTo(sx(traj[0].x), sy(traj[0].y))
        for (i in 1 until traj.size) path.lineTo(sx(traj[i].x), sy(traj[i].y))
        drawPath(path, Console.Cyan, style = Stroke(width = 4f, cap = StrokeCap.Round))
    }

    val here = pt(snap.east, snap.north)

    // return guidance
    if (snap.guidance.active) {
        drawLine(Console.Amber, here, pt(0.0, 0.0), 2f, pathEffect = PathEffect.dashPathEffect(floatArrayOf(12f, 10f)))
        val t = pt(snap.guidance.targetE, snap.guidance.targetN)
        drawLine(Console.Amber, here, t, 4f, cap = StrokeCap.Round)
        drawCircle(Console.Amber, 6f, t, style = Stroke(3f))
    }

    // demo ground truth (to visualise drift)
    snap.demoTruth?.let { drawCircle(Color.White.copy(alpha = 0.7f), 6f, pt(it.x, it.y), style = Stroke(2f)) }

    // start point
    val s0 = pt(0.0, 0.0)
    drawCircle(Console.Green, 9f, s0)
    drawCircle(Console.Green.copy(alpha = 0.3f), 18f, s0)

    // 2σ uncertainty ellipse
    val a = snap.sigmaE * snap.sigmaE; val c = snap.sigmaN * snap.sigmaN; val b = snap.covEN
    val mid = (a + c) / 2; val dif = sqrt(((a - c) / 2).pow(2) + b * b)
    val r1 = (2 * sqrt(max(mid + dif, 0.0)) * ppm).toFloat().coerceIn(4f, 2000f)
    val r2 = (2 * sqrt(max(mid - dif, 0.0)) * ppm).toFloat().coerceIn(4f, 2000f)
    val theta = Math.toDegrees(0.5 * atan2(2 * b, a - c)).toFloat()
    rotate(-theta, here) {
        drawOval(Console.Red.copy(alpha = 0.15f), here - Offset(r1, r2), Size(2 * r1, 2 * r2))
        drawOval(Console.Red.copy(alpha = 0.6f), here - Offset(r1, r2), Size(2 * r1, 2 * r2), style = Stroke(1.5f))
    }

    // heading pointer
    val h = Math.toRadians(snap.headingDeg)
    val dir = Offset(sin(h).toFloat(), -cos(h).toFloat())
    val perp = Offset(cos(h).toFloat(), sin(h).toFloat())
    val arrow = Path().apply {
        val tip = here + dir * 20f
        moveTo(tip.x, tip.y)
        val l = here - dir * 10f + perp * 11f; lineTo(l.x, l.y)
        val m = here - dir * 4f; lineTo(m.x, m.y)
        val r = here - dir * 10f - perp * 11f; lineTo(r.x, r.y)
        close()
    }
    drawPath(arrow, Console.Red)
    drawPath(arrow, Color.White, style = Stroke(2f))

    // north arrow
    val na = Offset(24f, size.height - 44f)
    drawLine(Console.Text, na + Offset(0f, 14f), na - Offset(0f, 14f), 2f)
    drawPath(Path().apply { moveTo(na.x, na.y - 18f); lineTo(na.x - 6f, na.y - 6f); lineTo(na.x + 6f, na.y - 6f); close() }, Console.Text)
    drawText(measurer, "N", na + Offset(-4f, 16f), TextStyle(color = Console.Text, fontSize = 9.sp, fontWeight = FontWeight.Bold))

    // scale bar
    val barM = listOf(1.0, 2.0, 5.0, 10.0, 20.0, 50.0).first { it * ppm >= 50 || it == 50.0 }
    val bx = size.width - 16f - (barM * ppm).toFloat(); val by = size.height - 18f
    drawLine(Console.Text, Offset(bx, by), Offset(size.width - 16f, by), 3f)
    drawLine(Console.Text, Offset(bx, by - 5f), Offset(bx, by + 5f), 2f)
    drawLine(Console.Text, Offset(size.width - 16f, by - 5f), Offset(size.width - 16f, by + 5f), 2f)
    drawText(measurer, "${barM.toInt()} m", Offset(bx, by - 18f), TextStyle(color = Console.Text, fontSize = 9.sp, fontFamily = FontFamily.Monospace))
}

/** Big turn arrow for return navigation. */
@Composable
fun TurnArrow(turnDeg: Double, arrived: Boolean, modifier: Modifier = Modifier) {
    Canvas(modifier) {
        val c = center
        val r = size.minDimension / 2f
        drawCircle(Console.Amber.copy(alpha = 0.12f), r, c)
        drawCircle(Console.Amber, r, c, style = Stroke(2f))
        if (arrived) { drawCircle(Console.Green, r * 0.45f, c); return@Canvas }
        rotate(turnDeg.toFloat(), c) {
            val p = Path().apply {
                moveTo(c.x, c.y - r * 0.8f)
                lineTo(c.x + r * 0.45f, c.y - r * 0.1f)
                lineTo(c.x + r * 0.16f, c.y - r * 0.1f)
                lineTo(c.x + r * 0.16f, c.y + r * 0.7f)
                lineTo(c.x - r * 0.16f, c.y + r * 0.7f)
                lineTo(c.x - r * 0.16f, c.y - r * 0.1f)
                lineTo(c.x - r * 0.45f, c.y - r * 0.1f)
                close()
            }
            drawPath(p, Console.Amber)
        }
    }
}
