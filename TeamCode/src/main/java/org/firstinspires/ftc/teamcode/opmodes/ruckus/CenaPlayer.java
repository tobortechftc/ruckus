package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import android.content.Context;
import android.media.MediaPlayer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

import java.io.File;
import java.io.IOException;
import android.media.MediaPlayer;
import android.net.Uri;

public class CenaPlayer {
    //The player handling the audio
    private static MediaPlayer mediaPlayer = null;
    //Start the wubs
    public static void start(Context context, Telemetry telemetry) {
        String song = "/sdcard/Download/ToborSong.mp3";
        File songFile = new File(song);
        if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context, Uri.fromFile(songFile));
        if (mediaPlayer != null) {
            mediaPlayer.seekTo(0);
            mediaPlayer.start();
        } else {
            telemetry.addData("MediaPlayer is not created for file:", "%s.", song);
            telemetry.update();
        }
    }

    //Stop the wubs
    public static void stop() {
        if (mediaPlayer != null) {
            mediaPlayer.stop();
            try { mediaPlayer.prepare(); }
            catch (IOException e) {}
        }
    }
}

