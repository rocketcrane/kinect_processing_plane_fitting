/*
Test goals:
1. capture color image
2. capture point cloud and get an averaged plane from it
*/
import KinectPV2.*;
import java.nio.*;

KinectPV2 kinect;

void setup() {
  kinect = new KinectPV2(this);
  kinect.enableColorImg(true);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);
  
  kinect.init();
}

void draw() {
}

void mousePressed() {
  getAndSaveImg();
  getPointCloud();
}

void keyPressed() {
  planeRANSAC(10.0, 0.6, 100);
}
