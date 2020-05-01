package com.wingnutengineering.selfbalancingrobotbt;

import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.DividerItemDecoration;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.ParcelUuid;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.Toast;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private ListView listView;
    private ArrayAdapter aAdapter;
    private BluetoothAdapter bAdapter = BluetoothAdapter.getDefaultAdapter();
    private Set<BluetoothDevice> pairedDevices;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        listView = (ListView) findViewById(R.id.PairedListView);

        if(bAdapter==null){
            Toast.makeText(getApplicationContext(),"Bluetooth Not Supported", Toast.LENGTH_SHORT).show();
        }
        else{
            pairedDevices = bAdapter.getBondedDevices();
            ArrayList list = new ArrayList();
            if(pairedDevices.size()>0){
                for(BluetoothDevice device: pairedDevices){
                    String devicename = device.getName();
                    String macAddress = device.getAddress();
                    list.add("Name: "+devicename+"\nMAC Address: "+macAddress);
                }
                listView = (ListView) findViewById(R.id.PairedListView);
                aAdapter = new ArrayAdapter(getApplicationContext(), android.R.layout.simple_list_item_1, list);
                listView.setAdapter(aAdapter);
            }
        }

        listView.setOnItemClickListener(new AdapterView.OnItemClickListener()
        {
            @Override
            public void onItemClick(AdapterView<?> adapter, View v, int position, long arg3)
            {
                String deviceString = (String) adapter.getItemAtPosition(position);
                ParcelUuid uuid;
                for(BluetoothDevice device: pairedDevices){
                    String devicename = device.getName();
                    String macAddress = device.getAddress();
                    String temp = "Name: "+devicename+"\nMAC Address: "+macAddress;
                    if(temp.equals(deviceString)){
                        uuid=device.getUuids()[0];
                        Intent intent = new Intent(v.getContext(), ControlActivity.class);
                        intent.putExtra("MacAddress", macAddress);
                        intent.putExtra("uuid", device.getUuids()[0].toString());
                        startActivity(intent);
                    }
                }
            }
        });
    }


}
