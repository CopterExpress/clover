package express.copter.cleverrc

import android.content.Context
import android.os.Build
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.view.View
import android.view.WindowManager
import android.webkit.JavascriptInterface
import kotlinx.android.synthetic.main.activity_main.*
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer

fun pack(x: Short, y: Short, z: Short, r: Short): ByteArray {
    val pump_on_buf: ByteBuffer = ByteBuffer.allocate(8)
    pump_on_buf.putShort(r)
    pump_on_buf.putShort(z)
    pump_on_buf.putShort(y)
    pump_on_buf.putShort(x)

    return pump_on_buf.array().reversedArray()
}

fun send(host: String, port: Int, data: ByteArray, senderPort: Int = 0): Boolean {
    var ret = false
    var socket: DatagramSocket? = null
    try {
        socket = DatagramSocket(senderPort)
        val address = InetAddress.getByName(host)
        val packet = DatagramPacket(data, data.size, address, port)
        socket.send(packet)
        ret = true
    } catch (e: Exception) {
        e.printStackTrace()
    } finally {
        socket?.close()
    }
    return ret
}

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        fullScreenCall()
        main_web.loadUrl("file:///android_asset/index.html")

        main_web.settings.apply {
            domStorageEnabled = true
            javaScriptEnabled = true
            loadWithOverviewMode = true
            useWideViewPort = true
            setSupportZoom(false)
        }

        main_web.addJavascriptInterface(WebAppInterface(this), "appInterface")
    }

    private fun fullScreenCall() {
        window.setFlags(
            WindowManager.LayoutParams.FLAG_FULLSCREEN,
            WindowManager.LayoutParams.FLAG_FULLSCREEN
        )
        if (Build.VERSION.SDK_INT < 19) {
            val v = this.window.decorView
            v.systemUiVisibility = View.GONE
        } else {
            //for higher api versions.
            val decorView = window.decorView
            val uiOptions = View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
            decorView.systemUiVisibility = uiOptions
        }
    }
}

class WebAppInterface(c: Context) {
    @JavascriptInterface
    public fun postMessage(message: String) {
        val data = JSONObject(message)
        send("255.255.255.255", 35602, pack(data.getInt("x").toShort(), data.getInt("y").toShort(), data.getInt("z").toShort(), data.getInt("r").toShort()))
    }
}