package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.io.StringReader;
import java.util.Calendar;
import java.util.Date;
import java.sql.Timestamp;

/**
 * Created by RobotAdmin on 3/10/2017.
 */

public class Logger {

    File TextFile;

    FileOutputStream filestream;

    PrintStream printStream;

    Logger(boolean setEnabled, String filenameSuffix) throws FileNotFoundException {
        Enabled = setEnabled;
        //Create Log File Named based on datetimestamp

        String filename = GetTimestamp() + filenameSuffix;

        String pathname = "/sdcard/FIRST/log/" + filename + ".txt";

        if(Enabled){
            TextFile = new File(pathname);

            printStream = new PrintStream(TextFile);

            filestream = new FileOutputStream(TextFile);
        }
    }

    //close the file

    public void printMessage(String source, String message){
        if(Enabled)
        {

            printStream.println(message + source + GetTimestamp());

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

    private String GetTimestamp(){
        Calendar calendar = Calendar.getInstance();

        Date now = calendar.getTime();

        Timestamp dateTimeStamp = new Timestamp(now.getTime());

        return dateTimeStamp.toString();

    }



}
