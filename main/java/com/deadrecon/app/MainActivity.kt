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
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.deadrecon.app.geo.GeoFormat
import com.deadrecon.app.geo.LatLon
import com.deadrecon.app.geo.LocalTangentPlane
import com.deadrecon.app.nav.NavSnapshot
import com.deadrecon.app.nav.NavigationEngine
import com.deadrecon.app.ui.Console
import com.deadrecon.app.ui.DeadReconScreen
import com.deadrecon.app.ui.ScreenActions
import com.deadrecon.app.ui.theme.Dead_ReconTheme
import com.deadrecon.app.vision.OpticalFlowEngine
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

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
