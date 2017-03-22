package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.Calendar;
import java.util.Date;
import java.sql.Timestamp;

import static java.lang.System.out;

/**
 * Created by RobotAdmin on 3/10/2017.
 */

public class Logger {

    File LoggerFile;
    File CameraImageFile;

    FileOutputStream filestream;

    PrintStream printStream;

    OutputStream CameraStream;

    Logger(boolean setEnabled, String filenameSuffix) throws FileNotFoundException {
        Enabled = setEnabled;
        //Create Log File Named based on datetimestamp

        String textFilename = filenameSuffix + GetTimestamp();

        String textPathname = "/sdcard/FIRST/log/" + textFilename + ".txt";

        String imagePathname = "/sdcard/FIRST/log/" + textFilename + ".jpg";

        if(Enabled){
            LoggerFile = new File(textPathname);
            printStream = new PrintStream(LoggerFile);
            filestream = new FileOutputStream(LoggerFile);

            CameraImageFile = new File(imagePathname);
            CameraStream = new FileOutputStream(CameraImageFile);
        }
    }

    //close the file

    public void printMessage(String source, String message){
        if(Enabled)
        {
            printStream.println(message + "     " + source + "     " + GetTimestamp());
        }
    }

    public void saveImage(Bitmap image) throws IOException {
        if(Enabled){
            image.compress(Bitmap.CompressFormat.JPEG, 100, CameraStream);
        }

        CameraStream.flush();
        CameraStream.close();
    }


    private boolean Enabled;

    private String GetTimestamp(){
        Calendar calendar = Calendar.getInstance();

        Date now = calendar.getTime();

        Timestamp dateTimeStamp = new Timestamp(now.getTime());

        return dateTimeStamp.toString();

    }



}
