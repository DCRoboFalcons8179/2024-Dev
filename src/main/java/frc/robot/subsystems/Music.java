package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
// Used for string array list
// import java.io.File;
// import java.io.IOException;
// import java.nio.file.Files;
import java.util.ArrayList;

public class Music {
    public String currentSong = "";
    Orchestra orchestra = new Orchestra();

    ArrayList<String> songList = new ArrayList<>();
    int songIndex;

    public Music() {
        // Nice puts it a pain if you add more music
        // try {
        // // Will have to change for robot directory prob lol
        // Files.list(new File("src/main/deploy/music").toPath()).forEach(path -> {
        // System.out.println(path);
        // String pathString = path.toString();
        // this.songList.add(pathString);
        // });
        // } catch (IOException e) {
        // e.printStackTrace();
        // }
        songList.add("Final_Fantasy_VII_Intro.chrp");
        songList.add("SuperMarioSuperMan.chrp");
    }

    public void loadMusic(int songIndex) {
        this.songIndex = songIndex;
        if (!currentSong.equals(songList.get(this.songIndex))) {
            orchestra.stop();
            orchestra.loadMusic("music/" + songList.get(this.songIndex));
            System.out.println("Music Loaded");
            currentSong = this.songList.get(this.songIndex);
        }
    }

    public void playMusic() {
        orchestra.play();
    }

    public void musicCheck() {
        if (orchestra.isPlaying())
            System.out.println("Playing");

        else if (!orchestra.isPlaying()) {
            System.out.println("Not playing");
        }
    }

    public void nextTrack() {
        this.songIndex += 1;
        loadMusic(this.songIndex);
    }

    public void backTrack() {
        this.songIndex -= 1;
        loadMusic(this.songIndex);
    }
}
