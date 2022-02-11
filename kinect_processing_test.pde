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
  
  size(1920,1080,P3D);
  lights();
  noStroke();
}

void draw() {
  ArrayList <PVector> pointCloud = getPointCloud();
  PVector[] bestPlane = planeRANSAC(pointCloud, 10.0, 0.6, 1000);
  
  //iteratively draw all the points as spheres
  for (int i = 0; i < pointCloud.size(); i++) {
    float[] p = pointCloud.get(i).array(); //get this point as a float array
    
    pushMatrix(); //saves default coordinate system
    translate(p[0],p[1],p[2]);
    sphere(10);
    popMatrix(); //restores default coordinate system
  }
}

void mousePressed() {
  getAndSaveImg();
}

void keyPressed() {
}
