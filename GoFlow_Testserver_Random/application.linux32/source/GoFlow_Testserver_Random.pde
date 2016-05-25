import processing.net.*;

Server myServer;
int port = 3233;
Client c;
String msg;

void setup() {
  myServer = new Server(this, port);
}

void draw() {
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
            double t = Math.random()*1.7+0.3;
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
          
        } else if (msg.equals("6") || msg.equals("12")) {
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