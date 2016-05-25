import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.net.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class GoFlow_Testserver_Random extends PApplet {



Server myServer;
int port = 3233;
Client c;
String msg;

public void setup() {
  myServer = new Server(this, port);
}

public void draw() {
  c = myServer.available();      // Get the next available client
  if (c != null) {
    if (c.available() > 0) {
      msg = c.readString();
      if (msg != "") {
        msg = msg.trim();
        if (msg.equals("1") || msg.equals("7")) {
          println("Starting the program!");
          delay(1000);
          double x = Math.random()*2-1;
          double y = Math.random()*2-1;
          String sx = Double.toString(x);
          String sy = Double.toString(y);
          print("send\n");
          c.write(sx + ":"+ sy);
          
        } else if (msg.equals("3") || msg.equals("9")) {
          println("Starting the program!");
          for (int i=0; i<10; i++) {
            double t = Math.random()*1.7f+0.3f;
            int dt = (int)Math.round(t*1000); 
            delay(dt);
            String st = Double.toString(t);
            print("send\n");
            c.write(st);
          }
          
        } else if (msg.equals("4") || msg.equals("10")) {
          println("Starting the program!");
          double t = Math.random()*3;
          int dt = (int)Math.round(t*1000);
          String st = Double.toString(t);
          delay(dt);
          print("send\n");
          c.write(st);
          
        } else if (msg.equals("6") || msg.equals("12")) {
          println("Starting the program!");
          double l = Math.random()*2;
          String sl = Double.toString(l);
          delay(1000);
          print("send\n");
          c.write(sl);
        
        } else if (msg.equals("11")) {
          println("Starting the program!");
          double t = Math.random()*5;
          int dt = (int)Math.round(t*1000);
          String st = Double.toString(t);
          delay(dt);
          print("send\n");
          c.write(st);
          
        } else if (msg.equals("stop")) { // ------------ Stop server and end program
          println("Server ended.");
          println("Program ended.");
          myServer.stop();
          System.exit(0);
        } 
      }
    }
  } else {
    //print("draw got null\n");
  }
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "GoFlow_Testserver_Random" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
