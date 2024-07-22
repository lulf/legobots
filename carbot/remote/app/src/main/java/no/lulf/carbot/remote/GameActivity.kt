package no.lulf.carbot.remote

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.content.Context
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import no.lulf.carbot.remote.ui.theme.CarbotremoteTheme
import android.app.Activity
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Intent
import android.content.pm.PackageManager
import android.util.Log
import android.view.MotionEvent
import android.view.View
import android.widget.Button
import android.widget.RelativeLayout
import androidx.core.app.ActivityCompat
import java.util.UUID


class GameActivity: Activity() {
    private val TAG = "GameActivity"

    private val gatt get() = BluetoothConnection.gatt
    private val motorChar get() = BluetoothConnection.motorChar
    private val servoChar get() = BluetoothConnection.servoChar

    private var motorValue: Byte = 0;
    private var servoValue: Byte = 0;
    private val writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE;
    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContentView(R.layout.activity_control_buttons)

        // Create buttons
        val buttonUp: Button = findViewById(R.id.buttonUp);
        val buttonDown: Button = findViewById(R.id.buttonDown);
        val buttonLeft: Button = findViewById(R.id.buttonLeft);
        val buttonRight: Button = findViewById(R.id.buttonRight);
        val buttonReset: Button = findViewById(R.id.buttonReset);

        // Set click listeners
        buttonUp.setOnClickListener { forward() }
        buttonDown.setOnClickListener { backward() }
        buttonLeft.setOnTouchListener({ v, event ->  left(event)});
        buttonRight.setOnTouchListener { v, event -> right(event)}
        buttonReset.setOnClickListener { reset() }


    }

    private fun forward() {
        motorValue = if (motorValue + 22 > Byte.MAX_VALUE) {
            Byte.MAX_VALUE
        } else {
            (motorValue + 22).toByte()
        };
        Log.v(TAG, "FORWARD " + motorValue)
        Log.v(TAG, "Char: "+ motorChar)

        motorChar?.let {
            Log.v(TAG, "WRITE IT");
            val status = gatt?.writeCharacteristic(it, byteArrayOf(motorValue), writeType)

            Log.v(TAG, "WRITE STATUS "+ status)
        }
    }

    private fun backward() {
        motorValue = if (motorValue - 22 < Byte.MIN_VALUE) {
            Byte.MIN_VALUE
        } else {
            (motorValue - 22).toByte()
        };
        Log.v(TAG, "BACKWARD " + motorValue)
        motorChar?.let {
            gatt?.writeCharacteristic(it, byteArrayOf(motorValue), writeType)
        }
    }

    private fun left(event: MotionEvent): Boolean {
        val value = when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                1
            }
            MotionEvent.ACTION_UP -> {
                0
            }
            MotionEvent.ACTION_CANCEL -> {
                0
            }
            else -> {
                return false;
            }
        }
        servoValue = value.toByte();
        Log.v(TAG, "LEFT " + servoValue)

        servoChar?.let {
            gatt?.writeCharacteristic(it, byteArrayOf(servoValue), writeType)
        }
        return true;
    }

    private fun right(event: MotionEvent): Boolean {
        val value = when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                -1
            }
            MotionEvent.ACTION_UP -> {
                0
            }
            MotionEvent.ACTION_CANCEL -> {
                0
            }
            else -> {
                return false;
            }
        }
        servoValue = value.toByte();
        Log.v(TAG, "RIGHT " + servoValue)

        servoChar?.let {
            gatt?.writeCharacteristic(it, byteArrayOf(servoValue), writeType)
        }
        return true;
    }

    private fun reset() {
        motorValue = 0
        servoValue = 0
        Log.v(TAG, "RESET")

        motorChar?.let {
            gatt?.writeCharacteristic(it, byteArrayOf(motorValue), writeType)
        }
        servoChar?.let {
            gatt?.writeCharacteristic(it, byteArrayOf(servoValue), writeType)
        }
    }
}