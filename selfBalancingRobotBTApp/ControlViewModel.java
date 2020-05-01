package com.wingnutengineering.selfbalancingrobotbt;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;

import androidx.lifecycle.ViewModel;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

public class ControlViewModel extends ViewModel {
    String address;
    String name;
    UUID uuid;
    InputStream inputStream;
    OutputStream outputStream;
    BluetoothDevice device;
    BluetoothSocket socket;

    public ControlViewModel(){}

    public void setUUID(String uuid){
        this.uuid = UUID.fromString(uuid);
    }

    public boolean connect(){
        BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
        this.device=btAdapter.getRemoteDevice(this.address);
        try{
            this.socket = device.createInsecureRfcommSocketToServiceRecord(this.uuid);
            socket.connect();
            outputStream = socket.getOutputStream();
            return true;
        }
        catch(Exception e){
           return false;
        }
    }

    public boolean isConnected() {
        if (socket == null){
            return false;
        }
        else{
            return socket.isConnected();
        }
    }
}
