package frc.robot.subsystems;
import com.ctre.phoenix6.Orchestra;


public class Music {
    public String currentSong = "";
    Orchestra orchestra = new Orchestra();

    public void loadMusic(String song) {
        if (!currentSong.equals(song)) {
            orchestra.stop();
            orchestra.loadMusic(song);
            System.out.println("Music Loaded");
            currentSong = song;
        }        
    }

    public void playMusic() {
        orchestra.play();
    }

    public void musicCheck() {
        if(orchestra.isPlaying()) System.out.println("Playing");
    
        else if (!orchestra.isPlaying()){
            System.out.println("Not playing");
        }
    }
}
