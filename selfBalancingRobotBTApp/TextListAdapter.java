package com.wingnutengineering.selfbalancingrobotbt;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import java.util.ArrayList;
import java.util.List;

public class TextListAdapter extends RecyclerView.Adapter<TextListViewHolder>{
    TextListViewHolder.ClickListener clickListener;
    List< NameAndStatusClass> list;
    Context context;

    TextListAdapter(Context context, TextListViewHolder.ClickListener clickListener){
        this.clickListener = clickListener;
        this.list = new ArrayList<NameAndStatusClass>();
        this.context = context;
    }

    @NonNull
    @Override
    public TextListViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        View view = LayoutInflater.from(parent.getContext()).inflate(R.layout.text_list,parent,false);
        TextListViewHolder vh = new TextListViewHolder(view,this.clickListener);

        return vh;
    }

    @Override
    public void onBindViewHolder(@NonNull TextListViewHolder holder, int position) {
        holder.statusTV.setText(list.get(position).getStatus());
        holder.nameTV.setText(list.get(position).getName());
    }

    @Override
    public int getItemCount() {
        return this.list.size();
    }

    public void addItem(NameAndStatusClass nAndS) {
        if(!contains(nAndS)){
            list.add(nAndS);
            this.notifyItemInserted(list.size()-1);
        }
    }

    public boolean contains(NameAndStatusClass nAndS){
        for(NameAndStatusClass n:list){
            if(n.getName().equals(n.getName())){
                return true;
            }
        }
        return false;
    }
}
