package com.wingnutengineering.selfbalancingrobotbt;

import androidx.appcompat.app.AppCompatActivity;
import androidx.lifecycle.ViewModelProvider;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.ParcelUuid;
import android.view.View;
import android.widget.SeekBar;
import android.widget.TextView;

import java.io.IOException;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class ControlActivity extends AppCompatActivity {
    ControlViewModel model;
    TextView tv;
    UUID uuid;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        model = new ViewModelProvider(this).get(ControlViewModel.class);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);

       if (model.isConnected()) {
           tv = (TextView) findViewById(R.id.testTV1);
           tv.setText("Connected to: " + model.device.getName());
           tv.setTextColor(getResources().getColor(R.color.green));
        }
        else {
            try {
                Intent extras = getIntent();
                model.address = extras.getStringExtra("MacAddress");
                model.setUUID(extras.getStringExtra("uuid"));
                if (model.connect()) {
                    tv = (TextView) findViewById(R.id.testTV1);
                    tv.setText("Connected to: " + model.device.getName());
                    tv.setTextColor(getResources().getColor(R.color.green));
                } else {
                    tv = (TextView) findViewById(R.id.testTV1);
                    tv.setText("Failed to connect.");
                    tv.setTextColor(getResources().getColor(R.color.red));
                }
            } catch (Exception e) {
                tv = (TextView) findViewById(R.id.testTV1);
                tv.setText("Load: " + e.getMessage());
                tv.setTextColor(getResources().getColor(R.color.red));
                e.printStackTrace();
            }
        }


        SeekBar rightSeekBar = findViewById(R.id.controlRightSeekBar);
        SeekBar leftSeekBar = findViewById(R.id.controlLeftSeekBar);

        rightSeekBar.setOnSeekBarChangeListener(
                new SeekBar.OnSeekBarChangeListener() {
                    @Override
                    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                        rightProgressChanged(progress);
                    }

                    @Override
                    public void onStartTrackingTouch(SeekBar seekBar) {

                    }

                    @Override
                    public void onStopTrackingTouch(SeekBar seekBar) {

                    }
                }

        );

        leftSeekBar.setOnSeekBarChangeListener(
                new SeekBar.OnSeekBarChangeListener() {
                    @Override
                    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                        leftProgressChanged(progress);
                    }

                    @Override
                    public void onStartTrackingTouch(SeekBar seekBar) {

                    }

                    @Override
                    public void onStopTrackingTouch(SeekBar seekBar) {

                    }
                }

        );
    }


    /*public void connect(){
        uuid = UUID.fromString(model.uuid);
        BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
        model.device=btAdapter.getRemoteDevice(model.address);
        try{
            model.socket = btDevice.createInsecureRfcommSocketToServiceRecord(uuid);
            socket.connect();
            tv = (TextView)findViewById(R.id.testTV1);
            tv.setText("Connected to: " + btDevice.getName());
            tv.setTextColor(getResources().getColor(R.color.green));
            outputStream = socket.getOutputStream();
        }
        catch(Exception e){
            tv = (TextView)findViewById(R.id.testTV1);
            tv.setText("Failed to connect.");
            tv.setText("Connect: " + e.getMessage());
            tv.setTextColor(getResources().getColor(R.color.red));
            e.printStackTrace();
        }
    }*/

    public void rightProgressChanged(int progress){
        int leftProgress = ((SeekBar) findViewById(R.id.controlLeftSeekBar)).getProgress();
        sendData(leftProgress, progress);
    }

    public void leftProgressChanged(int progress){
        int rightProgress = ((SeekBar) findViewById(R.id.controlRightSeekBar)).getProgress();
        sendData(progress, rightProgress);
    }

    public void sendData(int leftProgress, int rightProgess){
        try{
            String s = "L" + leftProgress + "R" + rightProgess + ":";
            model.outputStream.write(s.getBytes());
        }
        catch(IOException ioe){
            TextView tv = (TextView)findViewById(R.id.testTV1);
            tv.setText("failed");
        }

    }
}
