package com.wingnutengineering.selfbalancingrobotbt;

import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import org.w3c.dom.Text;

public class TextListViewHolder extends RecyclerView.ViewHolder {
    TextView nameTV;
    TextView statusTV;
    LinearLayout item;
    ClickListener clickListener;


    public TextListViewHolder(@NonNull View itemView, final ClickListener clickListener) {
        super(itemView);
        this.clickListener = clickListener;
        nameTV = (TextView) itemView.findViewById(R.id.TextListName);
        statusTV = (TextView) itemView.findViewById(R.id.TextListStatus);
        item = (LinearLayout) itemView.findViewById(R.id.TextListItem);
        item.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(clickListener != null){
                    clickListener.onClickListener(getAdapterPosition());
                }

            }
        });

    }

    public interface ClickListener{
        void onClickListener(int position);
    }
}
