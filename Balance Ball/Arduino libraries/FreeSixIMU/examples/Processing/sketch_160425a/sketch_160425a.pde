import org.openkinect.freenect.*; 
import org.openkinect.processing.*;
import blobDetection.*;
Kinect kinect;
PImage currentImage;

void setup() {
    // Big enough to display two images side by side
    size(1280, 500); 
    
    // Initialize the Kinect and its video feed
    kinect = new Kinect(this);
    kinect.initVideo();
    
    // Grab and display the current image
    currentImage = kinect.getVideoImage();
    image(currentImage, 0, 0);
  }