package com.deadrecon.app

import android.Manifest
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.saveable.rememberSaveable
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
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.deadrecon.app.ui.theme.Dead_ReconTheme
import java.nio.ByteBuffer
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import kotlin.math.*
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.pow

// ==================================================================================
// MainActivity.kt
// ==================================================================================

/**
 * Android shell: sensors, camera and UI wiring only. All fusion lives in [NavigationEngine]
 * (sensor thread); geographic conversion lives in [LocalTangentPlane] (UI side).
 */
class MainActivity : ComponentActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var sensorThread: HandlerThread? = null
    private var sensorHandler: Handler? = null
    private val mainHandler = Handler(Looper.getMainLooper())

    private val engine = NavigationEngine()                    // touched only on the sensor thread
    private val flow = OpticalFlowEngine()                      // touched only on the camera thread
    private val cameraExecutor: ExecutorService = Executors.newSingleThreadExecutor()
    private var lastFrameTs = 0L
    private var lastPublishMs = 0L

    // UI state
    private var snapshot by mutableStateOf(NavSnapshot())
    private var geo by mutableStateOf<LocalTangentPlane?>(null)

    private val demoTicker = object : Runnable {
        override fun run() {
            engine.demoTick(0.05)
            publish(force = true)
            if (engine.demoActive) sensorHandler?.postDelayed(this, 50)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager

        setContent {
            Dead_ReconTheme {
                var hasCamera by remember {
                    mutableStateOf(ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED)
                }
                val launcher = rememberLauncherForActivityResult(ActivityResultContracts.RequestPermission()) { hasCamera = it }
                LaunchedEffect(Unit) { if (!hasCamera) launcher.launch(Manifest.permission.CAMERA) }

                var latText by rememberSaveable { mutableStateOf("") }
                var lonText by rememberSaveable { mutableStateOf("") }
                var inputError by remember { mutableStateOf<String?>(null) }

                val s = snapshot
                val g = geo
                val current = g?.toGeo(s.east, s.north)
                val (status, statusColor) = when {
                    !s.aligned -> "ALIGNING · HOLD STILL" to Console.Amber
                    s.demoActive -> "DEMO WALK" to Console.Cyan
                    g == null -> "SET START POINT" to Console.Amber
                    s.guidance.active -> "RETURN NAV" to Console.Amber
                    else -> "TRACKING" to Console.Green
                }

                val actions = ScreenActions(
                    onLatChange = { latText = it; inputError = null },
                    onLonChange = { lonText = it; inputError = null },
                    onSetStart = {
                        val lat = GeoFormat.parse(latText, isLat = true)
                        val lon = GeoFormat.parse(lonText, isLat = false)
                        if (lat == null || lon == null) {
                            inputError = "Invalid coordinate (lat −90…90, lon −180…180)"
                        } else {
                            geo = LocalTangentPlane(LatLon(lat, lon))
                            onEngine { setOriginHere() }
                        }
                    },
                    onReset = {
                        onEngine { resetAll() }
                        flow.reset()
                        geo = null
                    },
                    onToggleReturn = { onEngine { if (returning) stopReturn() else startReturn() } },
                    onToggleDemo = {
                        onEngine {
                            if (demoActive) stopDemo()
                            else { startDemo(); sensorHandler?.post(demoTicker) }
                        }
                    },
                    onCalibrate = {
                        if (!s.calibrating) onEngine { startCalibration() }
                        else onEngine {
                            val k = finishCalibration(10.0)
                            mainHandler.post {
                                Toast.makeText(
                                    this@MainActivity,
                                    if (k == null) "Calibration failed: too few steps detected" else "Stride calibrated · K = %.3f".format(k),
                                    Toast.LENGTH_SHORT
                                ).show()
                            }
                        }
                    }
                )

                Scaffold(modifier = Modifier.fillMaxSize(), containerColor = Console.Bg) { pad ->
                    DeadReconScreen(
                        snap = s, status = status, statusColor = statusColor,
                        origin = g?.origin, current = current,
                        latText = latText, lonText = lonText, inputError = inputError,
                        actions = actions, modifier = Modifier.padding(pad)
                    ) {
                        if (hasCamera) {
                            CameraPreview(cameraExecutor) { processFrame(it) }
                        } else {
                            Box(Modifier.fillMaxSize()) {
                                Text("Camera permission required for visual odometry", color = Color.White, modifier = Modifier.align(Alignment.Center))
                            }
                        }
                    }
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        val thread = HandlerThread("DeadReconSensors").apply { start() }
        sensorThread = thread
        val handler = Handler(thread.looper)
        sensorHandler = handler
        val periodUs = 10_000 // 100 Hz
        listOf(Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_GYROSCOPE, Sensor.TYPE_MAGNETIC_FIELD).forEach { type ->
            sensorManager.getDefaultSensor(type)?.let { sensorManager.registerListener(this, it, periodUs, handler) }
        }
        if (engine.demoActive) handler.post(demoTicker)
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
        sensorHandler?.removeCallbacksAndMessages(null)
        sensorThread?.quitSafely()
        sensorThread = null
        sensorHandler = null
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraExecutor.shutdown()
    }

    // ---------------------------------------------------------------- sensors (sensor thread)

    override fun onSensorChanged(event: SensorEvent) {
        val v = event.values
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> engine.onAccelerometer(event.timestamp, v[0].toDouble(), v[1].toDouble(), v[2].toDouble())
            Sensor.TYPE_GYROSCOPE -> engine.onGyroscope(event.timestamp, v[0].toDouble(), v[1].toDouble(), v[2].toDouble())
            Sensor.TYPE_MAGNETIC_FIELD -> engine.onMagnetometer(event.timestamp, v[0].toDouble(), v[1].toDouble(), v[2].toDouble())
        }
        publish()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    /** Publishes a snapshot to Compose at ≤ 20 Hz. Must run on the sensor thread. */
    private fun publish(force: Boolean = false) {
        val now = SystemClock.uptimeMillis()
        if (!force && now - lastPublishMs < 50) return
        lastPublishMs = now
        val s = engine.snapshot()
        mainHandler.post { snapshot = s }
    }

    /** Runs a command on the sensor thread so the engine is never touched concurrently. */
    private fun onEngine(block: NavigationEngine.() -> Unit) {
        val h = sensorHandler
        if (h != null) h.post { engine.block(); publish(force = true) }
        else { engine.block(); snapshot = engine.snapshot() }
    }

    // ---------------------------------------------------------------- camera (camera thread)

    private fun processFrame(image: ImageProxy) {
        try {
            val plane = image.planes[0]
            val grid = flow.downsample(plane.buffer, image.width, image.height, plane.rowStride, plane.pixelStride)
            val result = flow.process(grid)
            val ts = image.imageInfo.timestamp
            val dt = if (lastFrameTs == 0L) 0.0 else (ts - lastFrameTs) / 1e9
            lastFrameTs = ts
            val rotation = image.imageInfo.rotationDegrees
            sensorHandler?.post { engine.onVisionFrame(result, dt, rotation, flow.focalPx) }
        } catch (e: Exception) {
            e.printStackTrace()
        } finally {
            image.close()
        }
    }
}

@Composable
fun CameraPreview(executor: ExecutorService, onFrame: (ImageProxy) -> Unit) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val previewView = remember { PreviewView(context).apply { scaleType = PreviewView.ScaleType.FILL_CENTER } }

    DisposableEffect(lifecycleOwner) {
        val future = ProcessCameraProvider.getInstance(context)
        future.addListener({
            val provider = future.get()
            val preview = Preview.Builder().build().also { it.setSurfaceProvider(previewView.surfaceProvider) }
            val analysis = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
                .also { it.setAnalyzer(executor) { img -> onFrame(img) } }
            try {
                provider.unbindAll()
                provider.bindToLifecycle(lifecycleOwner, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis)
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }, ContextCompat.getMainExecutor(context))
        onDispose { runCatching { future.get().unbindAll() } }
    }

    AndroidView(factory = { previewView }, modifier = Modifier.fillMaxSize())
}

// ==================================================================================
// nav/NavigationEngine.kt
// ==================================================================================

/** Immutable view of the navigation state, published to the UI ~20×/s. */
data class NavSnapshot(
    val aligned: Boolean = false,
    val east: Double = 0.0,
    val north: Double = 0.0,
    val velE: Double = 0.0,
    val velN: Double = 0.0,
    val sigmaE: Double = 0.0,
    val sigmaN: Double = 0.0,
    val covEN: Double = 0.0,
    val headingDeg: Double = 0.0,
    val mode: MotionMode = MotionMode.STATIONARY,
    val modeConfidence: Double = 0.0,
    val stepCount: Int = 0,
    val pathLength: Double = 0.0,
    val strideK: Double = 0.47,
    val calibrating: Boolean = false,
    val calibrationDistance: Double = 0.0,
    val visionConfidence: Double = 0.0,
    val visionActive: Boolean = false,
    val visionCalibrated: Boolean = false,
    val visionStill: Boolean = false,
    val visionVelE: Double = 0.0,
    val visionVelN: Double = 0.0,
    val visionDepth: Double = 0.0,
    val visionConsistency: Double = 0.0,
    val magReliability: Double = 0.0,
    val trajectory: List<Point2D> = listOf(Point2D(0.0, 0.0)),
    val explored: LongArray = LongArray(0),
    val exploredArea: Double = 0.0,
    val guidance: ReturnGuidance = ReturnGuidance(false),
    val demoActive: Boolean = false,
    val demoTruth: Point2D? = null,
) {
    val speed get() = hypot(velE, velN)
    val displacement get() = hypot(east, north)
}

/**
 * The whole sensor-fusion pipeline, free of Android types so it can be unit-tested on a JVM.
 * NOT thread-safe: call every method from the same (sensor) thread.
 *
 *  IMU ─► Orientation EKF ─► gravity removal ─► Position EKF ◄─ ZUPT
 *   │                                               ▲   ▲
 *   └► step detector ─► Weinberg stride ─► PDR ─────┘   │
 *   └► HMM motion classifier (selects models/updates)   │
 *  Camera ─► pyramidal LK ─► rotation-compensated, self-scaled visual velocity
 */
class NavigationEngine {
    val orientation = OrientationEkf()
    val position = PositionEkf()
    val steps = StepDetector()
    val classifier = MotionClassifier()
    private val returnNav = ReturnNavigator()

    var declinationDeg = 0.0

    // --- raw sensor bookkeeping
    private var startNs = 0L
    private var lastAccT = 0L
    private var lastGyroT = 0L
    private val lastAcc = DoubleArray(3)
    private val lastMag = DoubleArray(3)
    private var haveMag = false
    private val lastGyroRaw = DoubleArray(3)
    private val gyroRate = DoubleArray(3)
    private val accStats = RollingStats(150)
    private val gyroStats = RollingStats(150)
    private val strictAcc = RollingStats(30)
    private val strictGyro = RollingStats(30)
    private var stillSinceNs = 0L
    private var accCount = 0
    private var aligned = false
    private var headingDeg = 0.0

    // --- PDR
    private var stepCount = 0
    private var pathLength = 0.0
    private var lastStepT = 0L
    private var lastStepVelE = 0.0
    private var lastStepVelN = 0.0
    private var calibrating = false
    private var calLength = 0.0
    private var anchorE = 0.0
    private var anchorN = 0.0
    private var anchorT = 0L
    private val pending = ArrayList<PendingStep>()

    // --- vision
    private var logDepth = ln(1.5)
    private var visionConsistency = 0.0
    private var visionSamples = 0
    private var visionConf = 0.0
    private var visionActive = false
    private var visionCalibrated = false
    private var visionStillT = 0L
    private var lastVisionUpdateT = 0L
    private var visionVelE = 0.0
    private var visionVelN = 0.0

    // --- trajectory / coverage
    private val trajectory = ArrayList<Point2D>().apply { add(Point2D(0.0, 0.0)) }
    private val explored = HashSet<Long>()
    private var trajDirty = true
    private var cachedTraj: List<Point2D> = trajectory.toList()
    private var cachedExplored = LongArray(0)

    // --- demo walker
    var demoActive = false; private set
    private var demoE = 0.0
    private var demoN = 0.0
    private var demoIdx = 1
    private var demoStepAcc = 0.0
    private var demoClockNs = 0L
    private val rng = java.util.Random(7)
    private val demoSpeed = 1.3

    // ============================== sensor inputs ==============================

    fun onAccelerometer(t: Long, ax: Double, ay: Double, az: Double) {
        lastAcc[0] = ax; lastAcc[1] = ay; lastAcc[2] = az
        if (startNs == 0L) startNs = t
        if (!orientation.initialized) {
            if (haveMag || t - startNs > 1_500_000_000L) orientation.initialize(lastAcc, if (haveMag) lastMag else null)
            lastAccT = t
            return
        }
        val dt = if (lastAccT == 0L) 0.01 else (t - lastAccT) / 1e9
        lastAccT = t

        orientation.updateAccel(ax, ay, az)
        val norm = sqrt(ax * ax + ay * ay + az * az)
        accStats.add(norm); strictAcc.add(norm)
        val w = orientation.bodyToWorld(lastAcc)          // gravity is purely vertical → E/N are linear accel
        val step = steps.add(t, norm)

        if (++accCount % 5 == 0) runClassifier(t)
        if (!demoActive) headingDeg = (Math.toDegrees(orientation.travelBearing()) + 360) % 360
        if (demoActive) return

        val stationaryStrict = strictAcc.full && strictAcc.std < 0.12 && strictGyro.mean < 0.06
        stillSinceNs = if (stationaryStrict) (if (stillSinceNs == 0L) t else stillSinceNs) else 0L
        if (stillSinceNs != 0L && t - stillSinceNs > 800_000_000L) {
            orientation.learnGyroBias(lastGyroRaw[0], lastGyroRaw[1], lastGyroRaw[2])
        }

        val mode = classifier.mode
        val walking = mode == MotionMode.WALKING || mode == MotionMode.RUNNING
        val sigma = when (mode) {
            MotionMode.STATIONARY -> 0.3; MotionMode.WALKING -> 1.0; MotionMode.RUNNING -> 1.6
            MotionMode.CRAWLING -> 0.8; MotionMode.HANDLING -> 0.6
        }
        position.predict(w[0], w[1], dt, sigma, useAccel = !walking)

        if (step != null && aligned) {
            val gaitP = classifier.probability(MotionMode.WALKING) + classifier.probability(MotionMode.RUNNING)
            val confirmed = walking || gaitP > 0.35 || steps.cadence(t) >= 1.0
            applyStep(step.length, step.interval, orientation.travelBearing(), t, confirmed)
        }

        val visionStill = t - visionStillT < 300_000_000L
        val zupt = stationaryStrict ||
                classifier.probability(MotionMode.STATIONARY) > 0.75 ||
                (visionStill && strictAcc.std < 0.35 && strictGyro.mean < 0.25)
        if (zupt) position.zupt(0.01)

        recordTrajectory()
    }

    fun onGyroscope(t: Long, gx: Double, gy: Double, gz: Double) {
        lastGyroRaw[0] = gx; lastGyroRaw[1] = gy; lastGyroRaw[2] = gz
        if (lastGyroT != 0L && orientation.initialized) orientation.predict(gx, gy, gz, (t - lastGyroT) / 1e9)
        lastGyroT = t
        val b = orientation.gyroBias
        gyroRate[0] = gx - b[0]; gyroRate[1] = gy - b[1]; gyroRate[2] = gz - b[2]
        val n = sqrt(gyroRate[0].pow(2) + gyroRate[1].pow(2) + gyroRate[2].pow(2))
        gyroStats.add(n); strictGyro.add(n)
    }

    fun onMagnetometer(@Suppress("UNUSED_PARAMETER") t: Long, mx: Double, my: Double, mz: Double) {
        lastMag[0] = mx; lastMag[1] = my; lastMag[2] = mz; haveMag = true
        if (orientation.initialized) orientation.updateMag(mx, my, mz, Math.toRadians(declinationDeg))
    }

    /**
     * @param frameDt       seconds between this and the previous camera frame
     * @param rotationDeg   ImageInfo.rotationDegrees (image → upright display, clockwise)
     */
    fun onVisionFrame(r: OpticalFlowEngine.Result, frameDt: Double, rotationDeg: Int, focalPx: Double) {
        val t = lastAccT
        visionConf += 0.3 * ((if (r.valid) r.confidence else 0.0) - visionConf)
        if (!r.valid || frameDt <= 0.0 || frameDt > 0.2 || !orientation.initialized) { visionActive = false; return }

        // image axes → upright display axes (x right, y down) → body axes (x right, y up)
        val (dx, dy) = rotateCw(r.tx, r.ty, rotationDeg)
        var bx = dx / frameDt
        var by = -dy / frameDt
        // remove the flow a pure rotation of the camera would have produced
        bx -= focalPx * gyroRate[1]
        by += focalPx * gyroRate[0]
        val sRate = r.expansion / frameDt
        // camera looks along body −Z: lateral flow ↔ X/Y motion, expansion ↔ motion along −Z
        val unit = doubleArrayOf(-bx / focalPx, -by / focalPx, -sRate)   // velocity per metre of depth
        val w = orientation.bodyToWorld(unit)
        val uh = hypot(w[0], w[1])

        if (r.confidence > 0.35 && uh < 0.03 && abs(sRate) < 0.03) visionStillT = t

        // Self-supervised scale: learn scene depth from agreement with step-based velocity.
        val pdrSpeed = hypot(lastStepVelE, lastStepVelN)
        if (t - lastStepT < 1_000_000_000L && pdrSpeed > 0.3 && r.confidence > 0.35 && uh > 0.02) {
            val cos = (w[0] * lastStepVelE + w[1] * lastStepVelN) / (uh * pdrSpeed)
            visionConsistency += 0.05 * (cos - visionConsistency)
            if (cos > 0.5) {
                logDepth += 0.05 * (ln(pdrSpeed / uh) - logDepth)
                logDepth = logDepth.coerceIn(ln(0.3), ln(12.0))
                visionSamples++
            }
        }
        visionCalibrated = visionSamples >= 25 && visionConsistency > 0.55
        val depth = exp(logDepth)
        visionVelE = depth * w[0]; visionVelN = depth * w[1]

        visionActive = visionCalibrated && r.confidence > 0.3 &&
                classifier.mode != MotionMode.HANDLING && !demoActive
        if (visionActive) {
            val sigma = 0.12 + 0.6 * (1.0 - r.confidence)
            if (position.updateVelocity(visionVelE, visionVelN, sigma, gate = 9.21)) lastVisionUpdateT = t
        }
    }

    // ============================== commands ==============================

    /** Current position becomes (0,0) — the geographic start point. */
    fun setOriginHere() {
        position.resetPosition()
        trajectory.clear(); trajectory.add(Point2D(0.0, 0.0))
        explored.clear(); markExplored(0.0, 0.0)
        returnNav.stop()
        stepCount = 0; pathLength = 0.0
        anchorT = 0L; pending.clear()
        trajDirty = true
    }

    fun resetAll() {
        orientation.reset(); position.reset(); steps.reset(); classifier.reset()
        returnNav.stop(); demoActive = false
        startNs = 0L; lastAccT = 0L; lastGyroT = 0L; aligned = false; accCount = 0
        accStats.clear(); gyroStats.clear(); strictAcc.clear(); strictGyro.clear()
        logDepth = ln(1.5); visionConsistency = 0.0; visionSamples = 0; visionCalibrated = false
        calibrating = false
        setOriginHere()
    }

    fun startReturn() { returnNav.start(trajectory.toList()) }
    fun stopReturn() { returnNav.stop() }
    val returning get() = returnNav.active

    fun startCalibration() { calibrating = true; calLength = 0.0 }

    /** Finish "walk a known distance" calibration. Returns the new Weinberg K, or null. */
    fun finishCalibration(knownDistance: Double): Double? {
        calibrating = false
        if (calLength < 3.0) return null
        val k = (steps.weinbergK * knownDistance / calLength).coerceIn(0.30, 0.75)
        steps.weinbergK = k
        return k
    }

    fun startDemo() {
        demoActive = true
        position.reset()
        setOriginHere()
        demoE = 0.0; demoN = 0.0; demoIdx = 1; demoStepAcc = 0.0
    }

    fun stopDemo() { demoActive = false; returnNav.stop() }

    /** Advances the synthetic walker; its steps go through the real PDR → EKF path. */
    fun demoTick(dt: Double) {
        if (!demoActive) return
        demoClockNs += (dt * 1e9).toLong()
        val route = DemoMap.demoRoute
        var dx: Double; var dy: Double; var d: Double
        val moving: Boolean
        if (returnNav.active) {
            // Like a real rescuer: follow the on-screen arrow, which is computed from the *estimate*.
            val g = returnNav.update(position.east, position.north, headingDeg)
            dx = g.targetE - position.east; dy = g.targetN - position.north; d = hypot(dx, dy)
            moving = !g.arrived && d > 0.05
        } else {
            dx = route[demoIdx].x - demoE; dy = route[demoIdx].y - demoN; d = hypot(dx, dy)
            if (d < 0.3 && demoIdx < route.lastIndex) {
                demoIdx++
                dx = route[demoIdx].x - demoE; dy = route[demoIdx].y - demoN; d = hypot(dx, dy)
            }
            moving = d > 0.3
        }
        if (moving) {
            val stepLen = if (returnNav.active) demoSpeed * dt else min(demoSpeed * dt, d)
            demoE += dx / d * stepLen; demoN += dy / d * stepLen
            demoStepAcc += stepLen
            headingDeg = (Math.toDegrees(atan2(dx, dy)) + 360) % 360
        }
        position.predict(0.0, 0.0, dt, 1.0, useAccel = false)
        if (demoStepAcc >= 0.72) {
            val len = demoStepAcc * (1 + 0.03 * rng.nextGaussian())
            val hdg = Math.toRadians(headingDeg + 2.0 * rng.nextGaussian())
            applyStep(len, 0.72 / demoSpeed, hdg, demoClockNs)
            demoStepAcc = 0.0
        }
        if (!moving) position.zupt(0.01)
        recordTrajectory()
    }

    // ============================== output ==============================

    fun snapshot(): NavSnapshot {
        if (trajDirty) {
            cachedTraj = trajectory.toList()
            cachedExplored = explored.toLongArray()
            trajDirty = false
        }
        val g = returnNav.update(position.east, position.north, headingDeg)
        val mode = if (demoActive) (if (demoStepAcc > 0 || hypot(position.velE, position.velN) > 0.2) MotionMode.WALKING else MotionMode.STATIONARY) else classifier.mode
        // The step-anchored EKF covariance is locally consistent but ignores slowly-accumulating
        // stride-scale and heading bias, so add an empirical PDR drift term (~3 % of distance).
        val drift2 = (DRIFT_FRACTION * pathLength).pow(2)
        return NavSnapshot(
            aligned = aligned || demoActive,
            east = position.east, north = position.north,
            velE = position.velE, velN = position.velN,
            sigmaE = sqrt(position.P[0][0] + drift2), sigmaN = sqrt(position.P[1][1] + drift2), covEN = position.P[0][1],
            headingDeg = headingDeg,
            mode = mode, modeConfidence = if (demoActive) 1.0 else classifier.probability(mode),
            stepCount = stepCount, pathLength = pathLength, strideK = steps.weinbergK,
            calibrating = calibrating, calibrationDistance = calLength,
            visionConfidence = visionConf, visionActive = visionActive, visionCalibrated = visionCalibrated,
            visionStill = lastAccT - visionStillT < 300_000_000L,
            visionVelE = visionVelE, visionVelN = visionVelN, visionDepth = exp(logDepth),
            visionConsistency = visionConsistency,
            magReliability = orientation.magReliability,
            trajectory = cachedTraj, explored = cachedExplored, exploredArea = cachedExplored.size.toDouble(),
            guidance = g, demoActive = demoActive,
            demoTruth = if (demoActive) Point2D(demoE, demoN) else null
        )
    }

    // ============================== internals ==============================

    private fun runClassifier(t: Long) {
        classifier.update(MotionClassifier.Features(accStats.std, gyroStats.mean, steps.cadence(t)))
        if (!aligned && orientation.initialized && t - startNs > 2_000_000_000L) aligned = true
        if (demoActive) return
        // Nothing observed velocity recently (no steps, no vision) → gently damp it.
        val noSteps = t - lastStepT > 1_200_000_000L
        val noVision = t - lastVisionUpdateT > 500_000_000L
        if (noSteps && noVision && classifier.mode != MotionMode.STATIONARY) {
            position.updateVelocity(0.0, 0.0, if (classifier.mode == MotionMode.HANDLING) 0.15 else 0.4)
        }
    }

    private class PendingStep(val dE: Double, val dN: Double, val length: Double, val t: Long)

    /**
     * PDR update. Each confirmed step gives two measurements:
     *  - velocity  = stride / step interval along the heading
     *  - position  = position at previous step + stride vector (keeps distance exact, no corner lag)
     * Steps detected before the classifier confirmed walking are held briefly and credited later.
     */
    private fun applyStep(length: Double, interval: Double, bearingRad: Double, t: Long, confirmed: Boolean = true) {
        val dE = length * sin(bearingRad); val dN = length * cos(bearingRad)
        pending.removeAll { t - it.t > 2_500_000_000L }
        if (!confirmed) { pending.add(PendingStep(dE, dN, length, t)); return }

        var sumE = dE; var sumN = dN; var sumL = length; var n = 1
        for (p in pending) { sumE += p.dE; sumN += p.dN; sumL += p.length; n++ }
        pending.clear()

        val v = length / interval.coerceIn(0.3, 1.2)
        val vE = v * sin(bearingRad); val vN = v * cos(bearingRad)
        if (anchorT != 0L && t - anchorT < 2_500_000_000L) {
            position.updatePosition(anchorE + sumE, anchorN + sumN, 0.05 + 0.04 * n)
        } else if (n > 1) {
            position.shift(sumE - dE, sumN - dN)
        }
        position.updateVelocity(vE, vN, 0.2)
        anchorE = position.east; anchorN = position.north; anchorT = t

        lastStepT = t; lastStepVelE = vE; lastStepVelN = vN
        stepCount += n; pathLength += sumL
        if (calibrating) calLength += sumL
    }

    private fun recordTrajectory() {
        val last = trajectory.last()
        val e = position.east; val n = position.north
        if (hypot(e - last.x, n - last.y) > 0.05) {
            trajectory.add(Point2D(e, n))
            if (trajectory.size > 6000) { // decimate the oldest half
                val keep = ArrayList<Point2D>(4500)
                for (i in trajectory.indices) if (i >= 3000 || i % 2 == 0) keep.add(trajectory[i])
                trajectory.clear(); trajectory.addAll(keep)
            }
            markExplored(e, n)
            trajDirty = true
        }
    }

    /** Marks 1 m cells within ~2 m of the rescuer as explored (approximate sensing footprint). */
    private fun markExplored(e: Double, n: Double) {
        val cx = floor(e).toInt(); val cy = floor(n).toInt()
        for (i in -2..2) for (j in -2..2) {
            if (i * i + j * j > 5) continue
            explored.add(cellKey(cx + i, cy + j))
        }
    }

    private fun rotateCw(u: Double, v: Double, deg: Int): Pair<Double, Double> = when (((deg % 360) + 360) % 360) {
        90 -> -v to u
        180 -> -u to -v
        270 -> v to -u
        else -> u to v
    }

    companion object {
        const val DRIFT_FRACTION = 0.03
        fun cellKey(ix: Int, iy: Int): Long = (ix.toLong() shl 32) or (iy.toLong() and 0xFFFFFFFFL)
        fun cellX(key: Long): Int = (key shr 32).toInt()
        fun cellY(key: Long): Int = key.toInt()
    }
}

// ==================================================================================
// nav/OrientationEkf.kt
// ==================================================================================

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

// ==================================================================================
// nav/PositionEkf.kt
// ==================================================================================

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

// ==================================================================================
// nav/StepDetector.kt
// ==================================================================================

/**
 * Peak/valley step detector on the low-passed accelerometer magnitude, with Weinberg stride
 * estimation:  L = K · (a_max − a_min)^¼ .  K is user-calibratable ("walk a known 10 m").
 */
class StepDetector(var weinbergK: Double = 0.47) {

    data class Step(val timestampNs: Long, val length: Double, val interval: Double)

    private enum class Phase { PEAK, VALLEY }

    private var lp = 9.80665
    private var lastT = 0L
    private var phase = Phase.PEAK
    private var candPeak = 0.0
    private var candT = 0L
    private var valleyMin = Double.MAX_VALUE
    private var phaseStartT = 0L
    private var lastStepT = 0L
    private val recent = ArrayDeque<Long>()

    var peakThreshold = 10.55    // m/s² — peak must exceed ≈ g + 0.75
    var valleyThreshold = 9.55   // m/s² — must dip below this before the next peak

    fun reset() {
        lp = 9.80665; lastT = 0L; phase = Phase.PEAK; candPeak = 0.0
        valleyMin = Double.MAX_VALUE; lastStepT = 0L; recent.clear()
    }

    fun add(tNs: Long, accNorm: Double): Step? {
        if (lastT == 0L) { lastT = tNs; phaseStartT = tNs; lp = accNorm; return null }
        val dt = ((tNs - lastT) / 1e9).coerceIn(1e-4, 0.1)
        lastT = tNs
        val rc = 1.0 / (2 * PI * 3.0)          // 3 Hz low-pass
        lp += dt / (rc + dt) * (accNorm - lp)
        if (lp < valleyMin) valleyMin = lp

        while (recent.isNotEmpty() && tNs - recent.first() > 3_000_000_000L) recent.removeFirst()

        when (phase) {
            Phase.PEAK -> {
                if (lp > peakThreshold) {
                    if (lp > candPeak) { candPeak = lp; candT = tNs }
                } else if (candPeak > 0.0 && lp < candPeak - 0.45) {
                    val sinceLast = (candT - lastStepT) / 1e9
                    val result = if (lastStepT == 0L || sinceLast >= 0.28) {
                        val swing = (candPeak - valleyMin).coerceAtLeast(0.2)
                        val length = (weinbergK * swing.pow(0.25)).coerceIn(0.25, 1.5)
                        val interval = if (lastStepT == 0L || sinceLast > 2.0) 0.55 else sinceLast
                        lastStepT = candT
                        recent.addLast(candT)
                        Step(candT, length, interval)
                    } else null
                    candPeak = 0.0
                    valleyMin = lp
                    phase = Phase.VALLEY; phaseStartT = tNs
                    return result
                }
            }
            Phase.VALLEY -> {
                if (lp < valleyThreshold || (tNs - phaseStartT) > 1_500_000_000L) {
                    phase = Phase.PEAK; phaseStartT = tNs
                }
            }
        }
        return null
    }

    /** Steps per second over the last 3 s. */
    fun cadence(nowNs: Long): Double {
        while (recent.isNotEmpty() && nowNs - recent.first() > 3_000_000_000L) recent.removeFirst()
        return recent.size / 3.0
    }

    fun secondsSinceLastStep(nowNs: Long): Double =
        if (lastStepT == 0L) Double.MAX_VALUE else (nowNs - lastStepT) / 1e9
}

// ==================================================================================
// nav/ReturnNavigator.kt
// ==================================================================================

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

// ==================================================================================
// nav/MatrixMath.kt
// ==================================================================================

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

// ==================================================================================
// ai/MotionClassifier.kt
// ==================================================================================

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

// ==================================================================================
// vision/OpticalFlowEngine.kt
// ==================================================================================

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
        val inv = Mat.inv(ata) ?: return null
        return Mat.mulVec(inv, atb)
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

// ==================================================================================
// geo/GeoCoordinates.kt
// ==================================================================================

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

// ==================================================================================
// map/DemoMap.kt
// ==================================================================================

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

// ==================================================================================
// ui/DeadReconScreen.kt
// ==================================================================================

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

// ==================================================================================
// ui/MapCanvas.kt
// ==================================================================================

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
