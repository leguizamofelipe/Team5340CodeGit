package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

/**
 * Created by RobotAdmin on 3/10/2017.
 */

public class Logger {

    Logger(boolean setEnabled, String filenameSuffix){
        Enabled = setEnabled;
        //Create Log File Named based on datetimestamp
//        if filenameSuffix != Null ""
//                filename = datetimestamp + filenameSuffix
    }

    //close the file

    public void printMessage(String source, String message){
        if(Enabled)
        {
            //print to file timestamp "-" + source + ":" + message

        }
    }

    public void saveImage(Bitmap image){
        if(Enabled){
            //print to file timestamp + "-  SAVING IMAGE " + imageName
            //also save the actual image to imageName
        }
    }


    private boolean Enabled;



}
