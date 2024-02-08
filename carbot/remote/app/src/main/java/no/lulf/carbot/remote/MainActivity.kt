package no.lulf.carbot.remote

import android.Manifest
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import androidx.core.app.ActivityCompat
import java.util.UUID


class MainActivity : Activity() {
    private val TAG = "MainActivity"

    private val deviceName = "carbot"
    private val serviceUUID = UUID.fromString("00002000-b0cd-11ec-871f-d45ddf138840")
    private val motorUUID = UUID.fromString("00002001-b0cd-11ec-871f-d45ddf138840")
    private val servoUUID = UUID.fromString("00002002-b0cd-11ec-871f-d45ddf138840")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val permissions = arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_BACKGROUND_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.BLUETOOTH);

        for (permission in permissions) {
            if (ActivityCompat.checkSelfPermission(
                    this,
                    permission
            ) != PackageManager.PERMISSION_GRANTED
            ) {
                Log.v(TAG, "Missing permission "+ permission)
                ActivityCompat.requestPermissions(
                    this,
                    permissions, 123);
                return;
            }
        }

        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val bluetoothAdapter = bluetoothManager.adapter

        scanForBluetoothDevices(bluetoothAdapter)
    }


    var scanCallback: ScanCallback? = null
    private fun scanForBluetoothDevices(bluetoothAdapter: BluetoothAdapter) {
        Log.v(TAG, "Starting bluetooth scan")
        scanCallback = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult?) {
                Log.v(TAG,"Scan result")

                val device = result?.device

                if (device?.name == deviceName) {
                    Log.v(TAG, "Found matching device!")
                    // Stop scanning
                    stopScanning(bluetoothAdapter);

                    // Connect to the device
                    connectToDevice(bluetoothAdapter, device)
                }
            }
        };
        bluetoothAdapter.bluetoothLeScanner?.startScan(scanCallback);
    }

    private fun stopScanning(bluetoothAdapter: BluetoothAdapter) {
        bluetoothAdapter.bluetoothLeScanner?.stopScan(scanCallback)
        scanCallback = null // Clear the reference
    }

    private fun connectToDevice(adapter: BluetoothAdapter, device: BluetoothDevice) {
        Log.v(TAG, "Connecting to device " + device.name)
        device.connectGatt(this, true, object : BluetoothGattCallback() {
            override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
                super.onConnectionStateChange(gatt, status, newState)
                if (newState == BluetoothGatt.STATE_CONNECTED) {
                    Log.v(TAG, "Connected!")
                    gatt.discoverServices()
                }
            }

            override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
                super.onServicesDiscovered(gatt, status)
                val service: BluetoothGattService? = gatt.getService(serviceUUID)
                BluetoothConnection.gatt = gatt
                BluetoothConnection.motorChar = service?.getCharacteristic(motorUUID)
                BluetoothConnection.servoChar = service?.getCharacteristic(servoUUID)

                stopScanning(adapter)
                Log.v(TAG, "Discovered and retrieved services")
                val intent = Intent(this@MainActivity, GameActivity::class.java)
                startActivity(intent)
                finish()
            }
        });
    }
}