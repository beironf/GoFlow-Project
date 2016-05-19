import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import org.openkinect.freenect.*; 
import org.openkinect.freenect2.*; 
import org.openkinect.processing.*; 
import blobDetection.*; 
import java.awt.*; 
import java.util.*; 
import processing.net.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class KinectMotServer extends PApplet {

/// Daniel Shiffman
// All features test

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/









// Server variables
Server myServer;
int port = 3233;
Client c;
String msg;

Kinect2 kinect2;
boolean showImage = true;
float distanceX;
float distanceY;

// Variabler f\u00f6r boll
float redThreshold = 0.3f;
int redCount = 0;
int sumX = 0;
int sumY = 0;
float meanDepth;
int meanX;
int meanY;

// Variabler f\u00f6r m\u00e5l
int sumGoalX;
int sumGoalY;
float greenThreshold = 0.2f;
int greenCount;
float meanGoalDepth;
int meanGoalX;
int meanGoalY;



// Variabler f\u00f6r val av \u00f6vning
boolean rollToTarget = false;
boolean throwAgainstTarget = false;
boolean moveToCircle = false;
boolean fastTurns = false;
boolean doneWithExercise = false;



// Move To Circle Variables   ---------------------------------- CIRCLE
boolean isInCircle;
int startTime;
int countCircle;
boolean isDone;
float totalTime;

// Fast Turns Variables 
int greenCountLeft;
int greenCountRight;
int sumGoalXLeft;
int sumGoalXRight;
int sumGoalYLeft;
int sumGoalYRight;
float goalDepthLeft;
float goalDepthRight;
int meanGoalXLeft;
int meanGoalXRight;
int meanGoalYLeft;
int meanGoalYRight;


int nrOfTotalTurns = 10;
float redXPos;
float rightXPos;
float leftXPos;
float tStart;
float tid = 0;
boolean shouldPassRight;




// ----------- Methods -------------
public float calculateRedValue (int c) {
    return red(c)/255 - (green(c)/255 + blue(c)/255)/2;
}

public float calculateGreenValue (int c) {
    return green(c)/255 - (red(c)/255+blue(c)/255)/2;
}

public float calculateBlueValue (int c) {
    return blue(c)/255 - (red(c)/255+green(c)/255)/2;
}

public float distanceBetween (float rX, float rY, float rZ, float gX, float gY, float gZ) {
  float gXLengthPerPixel = tan(35.3f*PI/180)*gZ/256;
  float rXLengthPerPixel = tan(35.3f*PI/180)*rZ/256;
  float gYLengthPerPixel = tan(30*PI/180)*gZ/212;
  float rYLengthPerPixel = tan(30*PI/180)*rZ/212;
  rX = (rX-256)*rXLengthPerPixel*(-1);
  rY = (rY-212)*rYLengthPerPixel*(-1);
  gX = (gX-256)*gXLengthPerPixel*(-1);
  gY = (gY-212)*gYLengthPerPixel*(-1);
  //println("rx: " + rX + " ry: " + rY);
  return sqrt((rX-gX)*(rX-gX) + (rY-gY)*(rY-gY) + (rZ-gZ)*(rZ-gZ));
}

public float distanceBetweenX (float rX, float rZ, float gX, float gZ) {
  float gXLengthPerPixel = tan(35.3f*PI/180)*gZ/256;
  float rXLengthPerPixel = tan(35.3f*PI/180)*rZ/256;
  rX = (rX-256)*rXLengthPerPixel*(-1);
  gX = (gX-256)*gXLengthPerPixel*(-1);
  return abs(rX - gX);
}

public float distanceBetweenY (float rY, float rZ, float gY, float gZ) {
  float gYLengthPerPixel = tan(30*PI/180)*gZ/212;
  float rYLengthPerPixel = tan(30*PI/180)*rZ/212;
  rY = (rY-212)*rYLengthPerPixel*(-1);
  gY = (gY-212)*gYLengthPerPixel*(-1);
  return abs(rY - gY);
}


public float calculateXPosition (float rX, float rZ) {
  float lengthPerPixelX = tan(35.3f*PI/180)*rZ/256;
  rX = (rX-256)*lengthPerPixelX;
  return rX;
}

public void keyPressed() {
  if (key == CODED) {
    // Increase of decrease the threshold for green or red with the arrows
    if (keyCode == RIGHT) {
      redThreshold += 0.03f;
    } else if (keyCode == LEFT) {
      redThreshold -= 0.03f;
    } else if (keyCode == UP) {
      greenThreshold += 0.03f;
    } else if (keyCode == DOWN) {
      greenThreshold -= 0.03f;
    }
  }
  // Show image if 'i' is being pressed
  if (key == 'i') {
    if (!showImage) {
      showImage = true;
    } else {
      showImage = false;
    }
    // Start the time for Move To Circle if 
  } else if (key == 'a') {
      startTime = millis();
      isDone = false;     
  } else if (key == 'f') { // Start the excercies Fast Turns
    fastTurns = true;
  } else if (key == 't') { // Start the excercies Throw Against Target
    throwAgainstTarget = true;
  } else if (key == 'r') { // Start the excercies Roll To Target
    rollToTarget = true;
  } else if (key == 'm') { // Start the excercies Move To Circle 
    moveToCircle = true;
  } 
} 

public void setup() {
  
  kinect2 = new Kinect2(this);
  kinect2.initDepth();
  kinect2.initVideo();
  kinect2.initIR();
  kinect2.initRegistered();
  // Start all data
  kinect2.initDevice();
  frameRate(30); // camera limit
  
  myServer = new Server(this, port);
} 
 
public void draw() {
  background(0);
  PImage img = kinect2.getRegisteredImage();
  
  
  int[] depthRaw = kinect2.getRawDepth();
  img.loadPixels();
  // check distance to some points on the wall
  int HU = 512*50+256+100;
  int HN = 512*250+256+100;
  int VU = 512*50+256-100;
  int VN = 512*250+256-100;
  float precision = 5.0f;
  
  // Adjust the camera up or down
  fill(255);
  if ((depthRaw[VN]+depthRaw[HN])/2 > (depthRaw[VU]+depthRaw[HU])/2 + precision) {
    text("Vinkla upp kameran", 10, 405);
  } else if ((depthRaw[VN]+depthRaw[HN])/2 < (depthRaw[VU]+depthRaw[HU])/2 - precision) {
    text("Vinkla ner kameran", 10, 405);
  } else {
    text("", 10, 405);
  }
  // Adjust the camera left of right
  if ((depthRaw[VN]+depthRaw[VU])/2 > (depthRaw[HU]+depthRaw[HN])/2 + precision) {
    text("Vinkla kameran till v\u00e4nster", 200, 405);
  } else if ((depthRaw[VN]+depthRaw[VU])/2 < (depthRaw[VU]+depthRaw[HU])/2 - precision) {
    text("Vinkla kameran till h\u00f6ger", 200, 405);
  } else {
    text("", 200, 405);
  }
  for (int i = 0; i < 512*424; i += 1) {
    if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 

      // Set pixel to green
      img.pixels[i] = color(0, 255, 0);
    }
    if (calculateRedValue(img.pixels[i]) >= redThreshold) { 

      // Set pixel to red
      img.pixels[i] = color(255, 0, 0);
    }
  }
  text("Green Threshold: " + greenThreshold + "     Red Threshold: " + redThreshold, 10, 420);
  
  img.updatePixels();
  image(img, 0, 0, kinect2.depthWidth, kinect2.depthHeight);
  c = myServer.available();      // Get the next available client
  if (c != null) {
    if (c.available() > 0) {    // Are there any data incoming?
      msg = c.readString();     // Read the data
      if (msg != "") {
        msg = msg.trim();       // Get rid of blank spaces
        
        
        // Check wich exercise the app wants to run: 
        
        if (msg.equals("1") || msg.equals("7")) {   // ------------------------- Throw Against Target 
          boolean lastTime = false;
          doneWithExercise = false;
          println("Starting Throw Against Target!\n");
          float[][] distancesX = new float[2][150];
          float[][] distancesY = new float[2][150];
          int throwCount = 0;
          int nearest = 0;
          while (true) {
            // Clear variables
            sumGoalX = 0;
            sumGoalY = 0;
            sumX = 0;
            sumY = 0;
            redCount = 0;
            greenCount = 0;
            meanDepth = 0;
            meanGoalDepth = 0;
            meanGoalX = 0;
            meanGoalY = 0;
            meanX = 0;
            meanY = 0;
            distanceX = 0;
            distanceY = 0;
            
            // Set background color
            background(0);
            // Get video and depth values from Kinect
            img = kinect2.getRegisteredImage();
            depthRaw = kinect2.getRawDepth();
            img.loadPixels();
          
            // Find red and Green pixels
            for (int i = 0; i < 512*424; i += 1) {
                if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 
                  sumGoalX += i%512;
                  sumGoalY += i/512;
      
                  
                  greenCount++;
                  // Set pixel to green
                  img.pixels[i] = color(0, 255, 0);
                }
                if (calculateRedValue(img.pixels[i]) >= redThreshold) { 
                  sumX += i%512;
                  sumY += i/512;
      
                  redCount++;
                  // Set pixel to red
                  img.pixels[i] = color(255, 0, 0);
                }
            }
            
            // get depth on average x and y for green and red
            if (sumX != 0 && sumY != 0) {
              meanY = sumY/redCount;
              meanX = sumX/redCount;
              meanDepth = depthRaw[round(floor(meanY))*512 + round(meanX)];
            }
            if (sumGoalX != 0 && sumGoalY != 0) {
              meanGoalX = sumGoalX/greenCount;
              meanGoalY = sumGoalY/greenCount;
              meanGoalDepth = depthRaw[round(floor(meanGoalY))*512 + round(meanGoalX)];
            }
          
            // Ovning - "if ball hits the wall"
            if (meanDepth > meanGoalDepth - 500 && meanDepth < meanGoalDepth - 100 && distanceX == 0 && distanceY == 0 && meanGoalDepth != 0) {
              distancesX[0][throwCount] = meanGoalDepth - meanDepth;
              distancesY[0][throwCount] = meanGoalDepth - meanDepth;
              distancesX[1][throwCount] = distanceBetweenX(meanX, meanDepth, meanGoalX, meanGoalDepth);
              distancesY[1][throwCount] = distanceBetweenY(meanY, meanDepth, meanGoalY, meanGoalDepth);
              throwCount++;
              

              
            }
            
            
            if (lastTime == true && throwCount != 0 && redCount == 0) {
              for (int i = 0; i < throwCount; i++) {
                if (distancesX[0][i] == min(distancesX[0])) {
                  nearest = i;
                }
              }
              
              doneWithExercise = true;
            }
            
            if (throwCount != 0 && redCount == 0) {
              lastTime = true;
            }
            
            // Text on image
            text(
              "Framerate: " + PApplet.parseInt(frameRate) + "     Green Threshold: " + greenThreshold + "     Red Threshold: " + redThreshold, 10, 420);
            
            
            img.updatePixels();
            
            if (doneWithExercise) {
              println("send data\n");
              println("Minsta djup \u00e4r " + distancesX[0][nearest]/1000 + "meter");
              println(distancesX[1][nearest]/1000);
              println(distancesY[1][nearest]/1000);
              String msgToSendX = Float.toString(distancesX[1][nearest]/1000);
              String msgToSendY = Float.toString(distancesY[1][nearest]/1000);
              c.write(msgToSendX + ":" + msgToSendY);
              break;
            }
          }
        } else if (msg.equals("3") || msg.equals("9")) {  // ------------------- Fast Turns
          doneWithExercise = false;
          println("Starting Fast Turns!\n");
          // Clear variables
          int nrOfTurns = 0;
          boolean passed = false; // nrOfTurns can increase if true
          boolean haveStarted = false;
          while (true) {
            redCount = 0;
            greenCountLeft = 0;
            greenCountRight = 0;
            sumGoalXLeft = 0;
            sumGoalYRight = 0;
            sumGoalYLeft = 0;
            sumGoalXRight = 0;
            meanX = 0;
            meanY = 0;
            sumX = 0;
            sumY = 0;
            meanGoalXLeft = 0;
            meanGoalXRight = 0;
            meanGoalYLeft = 0;
            meanGoalXRight = 0;
           
            
            // Get video and depth values from Kinect
            img = kinect2.getRegisteredImage();
            depthRaw = kinect2.getRawDepth();
            img.loadPixels();
            
            // Step through every pixel and see if the redness > threshold or greeness > threshold
            for (int i = 0; i < 512*424; i++) {
              if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 
                // Save values
                if (i%512 > 256) {
                  sumGoalXRight += i%512;
                  sumGoalYRight += i/512;
                  greenCountRight++;
                } else {
                  sumGoalXLeft += i%512;
                  sumGoalYLeft += i/512;
                  greenCountLeft++;
                }
                
                // Set pixel to green
                img.pixels[i] = color(0, 255, 0);
              }
              if (calculateRedValue(img.pixels[i]) >= redThreshold) { 
                // Save values
                sumX += i%512;
                sumY += i/512;
                      
                redCount++;
                // Set pixel to red
                img.pixels[i] = color(255, 0, 0);
              }
            }
            
            // get depth
            if (redCount > 0) {
              meanY = sumY/redCount;
              meanX = sumX/redCount;
              
              meanDepth = depthRaw[round(floor(meanY))*512 + round(meanX)];
            }
            if (greenCountLeft > 0) {
              meanGoalXLeft = sumGoalXLeft/greenCountLeft;
              meanGoalYLeft = sumGoalYLeft/greenCountLeft;
              
              goalDepthLeft = depthRaw[round(floor(meanGoalYLeft))*512 + round(meanGoalXLeft)];
            }
            if (greenCountRight > 0) {
              meanGoalXRight = sumGoalXRight/greenCountRight;
              meanGoalYRight = sumGoalYRight/greenCountRight;
              
              goalDepthRight = depthRaw[round(floor(meanGoalYRight))*512 + round(meanGoalXRight)];
            }
            
            
            // Ovning
            if (greenCountLeft > 0 && greenCountRight > 0 && redCount > 0) {
              redXPos = calculateXPosition(meanX, meanDepth);
              leftXPos = calculateXPosition(meanGoalXLeft, goalDepthLeft);
              rightXPos = calculateXPosition(meanGoalXRight, goalDepthRight);
              
              if (nrOfTurns == 0 && redXPos < rightXPos && redXPos > leftXPos && haveStarted == false) {
                tStart = millis();
                haveStarted = true;
                shouldPassRight = true;
              }
              if (haveStarted == true && shouldPassRight == true && redXPos > rightXPos) {
                nrOfTurns++;
                tid = (millis() - tStart)/1000;
                tStart = millis();
                shouldPassRight = false;
                print("send data\n");
                String msgToSend = Float.toString(tid);
                c.write(msgToSend);
              }
               else if (haveStarted == true && shouldPassRight == false && redXPos < leftXPos) {
                  nrOfTurns++;
                  tid = (millis() - tStart)/1000;
                  tStart = millis();
                  shouldPassRight = true;
                  print("send data\n");
                  String msgToSend = Float.toString(tid);
                  c.write(msgToSend);
              }
            }
            

            img.updatePixels();
  
            if (nrOfTurns == 10) {
              break;
            }
          }
               
                
                
                
                
                
        } else if (msg.equals("4") || msg.equals("10")) {  // ------------------ Move To Circle
          startTime = millis();
          doneWithExercise = false;
          println("Starting Moving To Circle!\n");
          countCircle = 0;
          totalTime = 0;
          int nbrOfFrames = 270;
          float[] distanceCircle = new float[nbrOfFrames];
          for (int i = 0 ; i < nbrOfFrames; i++) {
            distanceCircle[i] = 1000;
          }
          
          while (true) {
            // Clear variables
            sumGoalX = 0;
            sumGoalY = 0;
            sumX = 0;
            sumY = 0;
            greenCount = 0;
            redCount = 0;
            meanDepth = 0;
            meanGoalDepth = 0;
            meanGoalX = 0;
            meanGoalY = 0;
            meanX = 0;
            meanY = 0;
            isInCircle = true;
          
            // If there have already been 300 measurements of the distance, make space for a new one.
            if (countCircle == nbrOfFrames) {
              for (int i = 1; i < nbrOfFrames; i++) {
                distanceCircle[i-1] = distanceCircle[i];
                countCircle = nbrOfFrames - 1;
              }
            }
            
            background(0);
            img = kinect2.getRegisteredImage();
            depthRaw = kinect2.getRawDepth();
            img.loadPixels();
             // Step through every pixel and see if the redness > threshold and greeness > threshold
            for (int i = 0; i < 512*424; i += 1) {
              if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 
                sumGoalX += i%512;
                sumGoalY += i/512;
                greenCount++;
                // Set pixel to green
                img.pixels[i] = color(0, 255, 0);
              }
              if (calculateRedValue(img.pixels[i]) >= redThreshold) { 
                sumX += i%512;
                sumY += i/512;
                redCount++;
                // Set pixel to red
                img.pixels[i] = color(255, 0, 0);
              }
            }
            
            // get depth
            if (sumX != 0 && sumY != 0) {
              meanY = sumY/redCount;
              meanX = sumX/redCount;
              meanDepth = depthRaw[round(floor(meanY))*512 + round(meanX)];
              
            }
            if (sumGoalX != 0 && sumGoalY != 0) {
              meanGoalX = sumGoalX/greenCount;
              meanGoalY = sumGoalY/greenCount;
              meanGoalDepth = depthRaw[round(floor(meanGoalY))*512 + round(meanGoalX)];
            }
            
            if (greenCount != 0 && redCount != 0) {
              distanceCircle[countCircle] = distanceBetween(meanX, meanY, meanDepth, meanGoalX, meanGoalY, meanGoalDepth);
              countCircle++;
            }
            
            
            // Check if the ball is inside the circle for 300 sequent measurements. 
            if (countCircle == nbrOfFrames && doneWithExercise == false) {
              for (int i = 0; i < nbrOfFrames; i++) { 
                if (distanceCircle[i] > 300) {
                  isInCircle = false;
                }
              }
              // If the above is true: print out the time and the excerices is done.
              if (isInCircle == true) {
                totalTime = (millis() - startTime - 2000);
                totalTime = totalTime/1000;
                doneWithExercise = true;
              }
            }
           
            
            img.updatePixels();
            if (doneWithExercise) {
              print("send data\n");
            
              
              String msgToSend = Float.toString(totalTime);
              c.write(msgToSend);
              break;
            }
          }

        
        } else if (msg.equals("6") || msg.equals("12")) {  // ------------------ Roll To Target 
          doneWithExercise = false;
          println("Starting Roll To Target!\n");
          float[] distances = new float[150];
          int count = 0;
          isDone = false;
          while (true) {
            sumGoalX = 0;
            sumGoalY = 0;
            sumX = 0;
            sumY = 0;
            greenCount = 0;
            redCount = 0;
            meanDepth = 0;
            meanGoalDepth = 0;
            meanGoalX = 0;
            meanGoalY = 0;
            meanX = 0;
            meanY = 0; 
          
            if (count == 150) {
              for (int i = 1; i < 150; i++) {
                distances[i-1] = distances[i];
                count = 149;
              }
            }
          
            background(0);
            // Get video and depth values from Kinect
            img = kinect2.getRegisteredImage();
            depthRaw = kinect2.getRawDepth();
            img.loadPixels();
            // Step through every pixel and see if the redness > threshold or greeness > threshold
            for (int i = 0; i < 512*424; i += 1) {
              if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 
                // Save values
                sumGoalX += i%512;
                sumGoalY += i/512;
                greenCount++;
                // Set pixel to green
                img.pixels[i] = color(0, 255, 0);
              }
              if (calculateRedValue(img.pixels[i]) >= redThreshold) { 
                // Save values
                sumX += i%512;
                sumY += i/512;
                redCount++;
                // Set pixel to red
                img.pixels[i] = color(255, 0, 0);
              }
            }
        
            // get depth for ball
            if (sumX != 0 && sumY != 0) {
              meanY = sumY/redCount;
              meanX = sumX/redCount;
              meanDepth = depthRaw[round(floor(meanY))*512 + round(meanX)];
          
            }
            
            // get depth for goal
            if (sumGoalX != 0 && sumGoalY != 0) {
              meanGoalX = sumGoalX/greenCount;
              meanGoalY = sumGoalY/greenCount;
              meanGoalDepth = depthRaw[round(floor(meanGoalY))*512 + round(meanGoalX)];
            }
            
            // If the ball is visbile, measure distance from the ball to the goal
            if (greenCount != 0 && redCount != 0) {
              distances[count] = distanceBetween(meanX, meanY, meanDepth, meanGoalX, meanGoalY, meanGoalDepth);
              count++;
            }
            
            // If at least 50 measurements have been done, check if the ball is in the same spot for all of them.
            if (count == 150 && max(distances) - min(distances) < 10 && !isDone) {
              println(distances[count-1]/10);
              isDone = true;
              doneWithExercise = true;
            }
            img.updatePixels();
            
            // The excercies is done
            if (doneWithExercise) {
              print("send data\n");
              String msgToSend = Float.toString(distances[count-1]/10);
              c.write(msgToSend);
              break;
            }
          }
                
            
        } else if (msg.equals("stop")) { // ------------ Stop server and end program
          println("Server ended.");
          println("Program ended.");
          myServer.stop();
          System.exit(0);
        }
      }
    }
  }
}
  public void settings() {  size(512, 424); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "KinectMotServer" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
