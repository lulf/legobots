package no.lulf.carbot.remote

import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic

object BluetoothConnection {
    var gatt: BluetoothGatt? = null
    var motorChar: BluetoothGattCharacteristic? = null
    var servoChar: BluetoothGattCharacteristic? = null
}
