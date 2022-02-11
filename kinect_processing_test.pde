/*
Test goals:
 1. capture color image
 2. capture point cloud and get an averaged plane from it
 
 Borrows some basic 3D rotation/coordinates code from https://stackoverflow.com/questions/28731442/detecting-set-of-planes-from-point-cloud
*/
import KinectPV2.*;
import java.nio.*;

KinectPV2 kinect;
float cR = 2; //compression ratio to reduce how far the points are from the center

void setup() {
  kinect = new KinectPV2(this);
  kinect.enableColorImg(true);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);

  kinect.init();

  size(1920, 1080, P3D);
  lights();
  noStroke();
  frameRate(30);
  delay(3000); //delay so that kinect has time to initialize
}

void draw() {
  background(255); //clears background
  //prints framerate
  textSize(20);
  text(frameRate,0,20);
  fill(0,0,0);
  
  ArrayList <PVector> pointCloud = getPointCloud();
  PVector[] bestPlane = planeRANSAC(pointCloud, 1.0, 0.8, 20);
  
  //draw from center and rotate with mouse
  translate(width * 0.5, height * 0.5, 0);
  rotateX(map(mouseY, 0, height, -PI, PI));
  rotateY(map(mouseX, 0, width, PI, -PI));

  //draw centered coordinate system
  drawAxes(80);
  //iteratively draw all the points as spheres
  for (int i = 0; i < pointCloud.size(); i++) {
    PVector p = pointCloud.get(i); //get this point
    pushMatrix(); //isolate coordinate system for pointCloud
    point(p.x/cR, p.y/cR, p.z/cR);//draw translated point, scaled by compression ratio
    //translate(p.x/cR, p.y/cR, p.z/cR); //can also use translate to draw objects, but much more CPU intensive
    popMatrix(); //end isolated pointCloud coordinate system
  }
  
  //draw best fit plane from RANSAC
  pushMatrix(); //isolate coordinate system for plane
  translate(bestPlane[0].x/cR, bestPlane[0].y/cR, bestPlane[0].z/cR); //draw a box at best fit plane center, scaled by compression ratio
  stroke(0,192,0);
  box(10);
  popMatrix(); //end isolated plane coordinate system
}

//zoom in and out with mouse wheel
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  cR = cR + e*0.2;
}

void mousePressed() {
  //getAndSaveImg();
}

void keyPressed() {
}
