/*
Test goals:
 1. capture color image
 2. capture point cloud and get an averaged plane from it
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
  //PVector[] bestPlane = planeRANSAC(pointCloud, 10.0, 0.6, 1000);
  //draw from centre and rotate with mouse
  translate(width * 0.5, height * 0.5, 0);
  rotateX(map(mouseY, 0, height, -PI, PI));
  rotateY(map(mouseX, 0, width, PI, -PI));

  //draw centred coordinate system
  drawAxes(100);

  
  //iteratively draw all the points as spheres
  for (int i = 0; i < pointCloud.size(); i++) {
    PVector p = pointCloud.get(i); //get this point
    pushMatrix(); //isolate coordinate system for pointCloud
    point(p.x/cR, p.y/cR, p.z/cR);//draw translated point
    //translate(p.x/cR, p.y/cR, p.z/cR); //can also use translate to draw objects, but much more CPU intensive
    popMatrix(); //end isolated coordinate system
  }
  //print("drew points\n");

  //iteratively draw all the points as spheres
  /*for (int i = 0; i < pointCloud.size(); i++) {
   float[] p = pointCloud.get(i).array(); //get this point as a float array
   
   pushMatrix(); //saves default coordinate system
   translate(p[0],p[1],p[2]);
   sphere(10);
   popMatrix(); //restores default coordinate system
   }*/
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
