package com.deadrecon.app.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.deadrecon.app.ai.MotionMode
import com.deadrecon.app.geo.GeoFormat
import com.deadrecon.app.geo.LatLon
import com.deadrecon.app.nav.NavSnapshot
import kotlin.math.abs
import kotlin.math.atan2

object Console {
    val Bg = Color(0xFF080C0B)
    val Panel = Color(0xFF101715)
    val Line = Color(0xFF22302A)
    val Text = Color(0xFFD9E4DE)
    val Dim = Color(0xFF7D8E86)
    val Amber = Color(0xFFFFB020)
    val Green = Color(0xFF3DDC84)
    val Red = Color(0xFFFF5252)
    val Cyan = Color(0xFF4FC3F7)
    val MapBg = Color(0xFF0B1210)
    val Grid = Color(0xFF15211D)
    val Wall = Color(0xFF5B6A64)
    val Floor = Color(0xFF1B2723)
    val RoomFloor = Color(0xFF1F2C33)
    val SafeFloor = Color(0xFF173226)
    val HazardFloor = Color(0xFF33191A)
    val MapLabel = Color(0xFF8FA39A)
}

private val Mono = FontFamily.Monospace

data class ScreenActions(
    val onLatChange: (String) -> Unit,
    val onLonChange: (String) -> Unit,
    val onSetStart: () -> Unit,
    val onReset: () -> Unit,
    val onToggleReturn: () -> Unit,
    val onToggleDemo: () -> Unit,
    val onCalibrate: () -> Unit,
)

@Composable
fun DeadReconScreen(
    snap: NavSnapshot,
    status: String,
    statusColor: Color,
    origin: LatLon?,
    current: LatLon?,
    latText: String,
    lonText: String,
    inputError: String?,
    actions: ScreenActions,
    modifier: Modifier = Modifier,
    camera: @Composable () -> Unit,
) {
    Column(
        modifier
            .fillMaxSize()
            .background(Console.Bg)
            .verticalScroll(rememberScrollState())
            .padding(horizontal = 10.dp, vertical = 8.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Header(status, statusColor, snap)

        // 2. Live camera
        Box(
            Modifier.fillMaxWidth().height(170.dp)
                .border(1.dp, Console.Line, RoundedCornerShape(10.dp))
                .background(Color.Black, RoundedCornerShape(10.dp))
        ) {
            camera()
            val visionLabel = when {
                snap.visionActive -> "VISION FUSED" to Console.Green
                snap.visionStill -> "VISION: STILL" to Console.Cyan
                snap.visionConfidence > 0.3 && !snap.visionCalibrated -> "VISION: LEARNING SCALE" to Console.Amber
                snap.visionConfidence > 0.15 -> "VISION: TRACKING" to Console.Amber
                else -> "VISION: SEARCHING" to Console.Dim
            }
            Column(
                Modifier.align(Alignment.BottomStart).padding(8.dp)
                    .background(Color.Black.copy(alpha = 0.55f), RoundedCornerShape(6.dp)).padding(6.dp)
            ) {
                Text(visionLabel.first, color = visionLabel.second, fontSize = 11.sp, fontFamily = Mono, fontWeight = FontWeight.Bold)
                ConfidenceBar(snap.visionConfidence, Modifier.width(120.dp).padding(top = 3.dp))
                Text("conf %3.0f%%".format(snap.visionConfidence * 100), color = Console.Text, fontSize = 10.sp, fontFamily = Mono)
            }
            Text(
                "${snap.mode.label} %.0f%%".format(snap.modeConfidence * 100),
                color = modeColor(snap.mode), fontSize = 11.sp, fontFamily = Mono, fontWeight = FontWeight.Bold,
                modifier = Modifier.align(Alignment.TopEnd).padding(8.dp)
                    .background(Color.Black.copy(alpha = 0.55f), RoundedCornerShape(6.dp)).padding(horizontal = 6.dp, vertical = 3.dp)
            )
        }

        // 3. Trajectory map
        Box(
            Modifier.fillMaxWidth().height(340.dp)
                .border(1.dp, Console.Line, RoundedCornerShape(10.dp))
        ) {
            MapPanel(snap, showDemoMap = true, modifier = Modifier.fillMaxSize())
        }

        if (snap.guidance.active) ReturnPanel(snap)

        CoordinatePanel(snap, origin, current, latText, lonText, inputError, actions)
        TelemetryPanel(snap)
        ControlPanel(snap, origin != null, actions)

        Text(
            "Dead-reckoned estimates — not GPS. Accuracy degrades with distance travelled.",
            color = Console.Dim, fontSize = 10.sp, fontFamily = Mono,
            modifier = Modifier.padding(vertical = 6.dp)
        )
    }
}

@Composable
private fun Header(status: String, statusColor: Color, snap: NavSnapshot) {
    Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Column(Modifier.weight(1f)) {
            Text("DEAD RECON", color = Console.Text, fontSize = 22.sp, fontWeight = FontWeight.Black, fontFamily = Mono, letterSpacing = 3.sp)
            Text("GPS-DENIED NAVIGATION", color = Console.Dim, fontSize = 10.sp, fontFamily = Mono, letterSpacing = 2.sp)
        }
        Column(horizontalAlignment = Alignment.End) {
            Row(
                Modifier.border(1.dp, statusColor, RoundedCornerShape(20.dp)).padding(horizontal = 10.dp, vertical = 4.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Box(Modifier.size(8.dp).background(statusColor, CircleShape))
                Spacer(Modifier.width(6.dp))
                Text(status, color = statusColor, fontSize = 11.sp, fontFamily = Mono, fontWeight = FontWeight.Bold)
            }
            Text(
                "COMPASS " + when {
                    snap.magReliability > 0.7 -> "OK"
                    snap.magReliability > 0.35 -> "WEAK"
                    else -> "DISTURBED"
                },
                color = if (snap.magReliability > 0.7) Console.Dim else Console.Amber,
                fontSize = 9.sp, fontFamily = Mono, modifier = Modifier.padding(top = 3.dp)
            )
        }
    }
}

@Composable
private fun ReturnPanel(snap: NavSnapshot) {
    val g = snap.guidance
    Panel("RETURN NAVIGATION") {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TurnArrow(g.turnDeg, g.arrived, Modifier.size(84.dp))
            Spacer(Modifier.width(14.dp))
            Column {
                val instruction = when {
                    g.arrived -> "AT START POINT"
                    abs(g.turnDeg) < 15 -> "STRAIGHT AHEAD"
                    abs(g.turnDeg) > 150 -> "TURN AROUND"
                    g.turnDeg > 0 -> "TURN RIGHT %.0f°".format(g.turnDeg)
                    else -> "TURN LEFT %.0f°".format(-g.turnDeg)
                }
                Text(instruction, color = if (g.arrived) Console.Green else Console.Amber, fontSize = 18.sp, fontWeight = FontWeight.Bold, fontFamily = Mono)
                Text("waypoint  %.1f m @ %03.0f°".format(g.distanceToWaypoint, g.bearingToWaypointDeg), color = Console.Text, fontSize = 12.sp, fontFamily = Mono)
                Text("retrace   %.1f m to start".format(g.pathRemaining), color = Console.Text, fontSize = 12.sp, fontFamily = Mono)
                Text("direct    %.1f m".format(g.straightLineToStart), color = Console.Dim, fontSize = 12.sp, fontFamily = Mono)
            }
        }
    }
}

@Composable
private fun CoordinatePanel(
    snap: NavSnapshot, origin: LatLon?, current: LatLon?,
    latText: String, lonText: String, inputError: String?, actions: ScreenActions
) {
    Panel("COORDINATES") {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            CoordField("Initial latitude", latText, actions.onLatChange, Modifier.weight(1f))
            CoordField("Initial longitude", lonText, actions.onLonChange, Modifier.weight(1f))
        }
        if (inputError != null) Text(inputError, color = Console.Red, fontSize = 11.sp, fontFamily = Mono)
        Button(
            onClick = actions.onSetStart,
            modifier = Modifier.fillMaxWidth().padding(top = 6.dp),
            colors = ButtonDefaults.buttonColors(containerColor = Console.Green, contentColor = Color.Black),
            shape = RoundedCornerShape(8.dp)
        ) { Text(if (origin == null) "SET START POINT" else "RE-SET START HERE", fontFamily = Mono, fontWeight = FontWeight.Bold) }

        Spacer(Modifier.height(8.dp))
        if (origin == null || current == null) {
            Text("Enter the approximate start coordinate, stand at it, then press SET START POINT.",
                color = Console.Dim, fontSize = 11.sp, fontFamily = Mono)
        } else {
            KV("START LAT", GeoFormat.lat(origin.latitude))
            KV("START LON", GeoFormat.lon(origin.longitude))
            ConsoleDivider()
            KV("CURRENT LAT", GeoFormat.lat(current.latitude), Console.Amber, big = true)
            KV("CURRENT LON", GeoFormat.lon(current.longitude), Console.Amber, big = true)
            KV("DMS", "${GeoFormat.dms(current.latitude, true)}  ${GeoFormat.dms(current.longitude, false)}")
            ConsoleDivider()
            KV("NORTH", signed(snap.north) + " m")
            KV("EAST", signed(snap.east) + " m")
            KV("TOTAL", "%.2f m".format(snap.displacement))
            val toStart = (Math.toDegrees(atan2(-snap.east, -snap.north)) + 360) % 360
            KV("START IS", "%.1f m @ %03.0f° %s".format(snap.displacement, toStart, GeoFormat.cardinal(toStart)))
            KV("± (2σ)", "%.1f m".format(2 * maxOf(snap.sigmaE, snap.sigmaN)), Console.Dim)
        }
    }
}

@Composable
private fun TelemetryPanel(s: NavSnapshot) {
    Panel("NAVIGATION TELEMETRY") {
        Row {
            Column(Modifier.weight(1f)) {
                KV("X (E)", "%.2f m".format(s.east))
                KV("Y (N)", "%.2f m".format(s.north))
                KV("Vx", "%.2f m/s".format(s.velE))
                KV("Vy", "%.2f m/s".format(s.velN))
                KV("SPEED", "%.2f m/s".format(s.speed))
                KV("HEADING", "%03.0f° %s".format(s.headingDeg, GeoFormat.cardinal(s.headingDeg)))
                KV("MAG", "%.0f%%".format(s.magReliability * 100))
            }
            Spacer(Modifier.width(10.dp))
            Column(Modifier.weight(1f)) {
                KV("FLOW Vx", "%.2f m/s".format(s.visionVelE))
                KV("FLOW Vy", "%.2f m/s".format(s.visionVelN))
                KV("VIS CONF", "%.0f%%".format(s.visionConfidence * 100))
                KV("VIS SCALE", "%.1f m %s".format(s.visionDepth, if (s.visionCalibrated) "✓" else "…"))
                KV("STEPS", "${s.stepCount}")
                KV("PATH", "%.1f m".format(s.pathLength))
                KV("STRIDE K", "%.3f".format(s.strideK))
            }
        }
        KV("EXPLORED", "%.0f m²".format(s.exploredArea))
    }
}

@Composable
private fun ControlPanel(s: NavSnapshot, hasStart: Boolean, a: ScreenActions) {
    Panel("CONTROLS") {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            ConsoleButton(
                if (s.guidance.active) "STOP RETURN" else "RETURN TO START",
                Console.Amber, Modifier.weight(1f), enabled = hasStart, filled = s.guidance.active, onClick = a.onToggleReturn
            )
            ConsoleButton("RESET NAV", Console.Red, Modifier.weight(1f), onClick = a.onReset)
        }
        Spacer(Modifier.height(6.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            ConsoleButton(
                if (s.calibrating) "DONE · 10 m (%.1f)".format(s.calibrationDistance) else "CALIBRATE STRIDE",
                Console.Cyan, Modifier.weight(1f), filled = s.calibrating, onClick = a.onCalibrate
            )
            ConsoleButton(
                if (s.demoActive) "STOP DEMO" else "DEMO WALK",
                Console.Text, Modifier.weight(1f), enabled = hasStart, filled = s.demoActive, onClick = a.onToggleDemo
            )
        }
        if (s.calibrating) Text("Walk exactly 10 m in a straight line, then press DONE.", color = Console.Cyan, fontSize = 11.sp, fontFamily = Mono, modifier = Modifier.padding(top = 6.dp))
    }
}

// ---------------------------------------------------------------- building blocks

@Composable
private fun Panel(title: String, content: @Composable ColumnScope.() -> Unit) {
    Column(
        Modifier.fillMaxWidth()
            .background(Console.Panel, RoundedCornerShape(10.dp))
            .border(1.dp, Console.Line, RoundedCornerShape(10.dp))
            .padding(10.dp)
    ) {
        Text(title, color = Console.Amber, fontSize = 11.sp, fontFamily = Mono, fontWeight = FontWeight.Bold, letterSpacing = 2.sp)
        Spacer(Modifier.height(6.dp))
        content()
    }
}

@Composable
private fun KV(k: String, v: String, color: Color = Console.Text, big: Boolean = false) {
    Row(Modifier.fillMaxWidth().padding(vertical = 1.dp)) {
        Text(k, color = Console.Dim, fontSize = 11.sp, fontFamily = Mono, modifier = Modifier.width(92.dp))
        Text(v, color = color, fontSize = if (big) 14.sp else 12.sp, fontFamily = Mono, fontWeight = if (big) FontWeight.Bold else FontWeight.Normal)
    }
}

@Composable
private fun ConsoleDivider() {
    Box(Modifier.fillMaxWidth().padding(vertical = 5.dp).height(1.dp).background(Console.Line))
}

@Composable
private fun CoordField(label: String, value: String, onChange: (String) -> Unit, modifier: Modifier) {
    OutlinedTextField(
        value = value,
        onValueChange = onChange,
        label = { Text(label, fontSize = 10.sp, fontFamily = Mono) },
        placeholder = { Text("e.g. 12.9716", fontSize = 12.sp, fontFamily = Mono) },
        singleLine = true,
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Text),
        textStyle = LocalTextStyle.current.copy(fontFamily = Mono, fontSize = 13.sp, color = Console.Text),
        colors = OutlinedTextFieldDefaults.colors(
            focusedBorderColor = Console.Amber, unfocusedBorderColor = Console.Line,
            focusedLabelColor = Console.Amber, unfocusedLabelColor = Console.Dim, cursorColor = Console.Amber
        ),
        modifier = modifier
    )
}

@Composable
private fun ConsoleButton(
    text: String, color: Color, modifier: Modifier, enabled: Boolean = true, filled: Boolean = false, onClick: () -> Unit
) {
    OutlinedButton(
        onClick = onClick, enabled = enabled, modifier = modifier, shape = RoundedCornerShape(8.dp),
        colors = ButtonDefaults.outlinedButtonColors(
            containerColor = if (filled) color.copy(alpha = 0.2f) else Color.Transparent,
            contentColor = color, disabledContentColor = Console.Dim
        ),
        border = androidx.compose.foundation.BorderStroke(1.dp, if (enabled) color else Console.Line),
        contentPadding = PaddingValues(horizontal = 6.dp, vertical = 10.dp)
    ) { Text(text, fontSize = 11.sp, fontFamily = Mono, fontWeight = FontWeight.Bold, maxLines = 1) }
}

@Composable
private fun ConfidenceBar(v: Double, modifier: Modifier) {
    Box(modifier.height(5.dp).background(Console.Line, RoundedCornerShape(3.dp))) {
        Box(
            Modifier.fillMaxHeight().fillMaxWidth(v.toFloat().coerceIn(0f, 1f))
                .background(if (v > 0.5) Console.Green else if (v > 0.2) Console.Amber else Console.Red, RoundedCornerShape(3.dp))
        )
    }
}

private fun modeColor(m: MotionMode) = when (m) {
    MotionMode.STATIONARY -> Console.Cyan
    MotionMode.WALKING, MotionMode.RUNNING -> Console.Green
    MotionMode.CRAWLING -> Console.Amber
    MotionMode.HANDLING -> Console.Dim
}

private fun signed(v: Double) = (if (v >= 0) "+" else "−") + "%.2f".format(abs(v))
