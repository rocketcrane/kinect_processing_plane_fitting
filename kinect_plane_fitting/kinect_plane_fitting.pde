/*
LICENSE & COPYRIGHT:
Copyright© 2022, Lingxiu C Zhang (https://github.com/rocketcrane/kinect_processing_plane_fitting)

Released under GPL-3.0-or-later
This file is part of the kinect_plane_fitting program.
Kinect_plane_fitting is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/
 
/*
This is a simple plane-fitting program for the Kinect V2 point cloud, using a RANSAC-like algorithm.
It relies on the KinectPV2 library to interface with the Kinect, and the PeasyCam library to help with 3D rendering and camera movement.
Install those two libraries, plug in a Kinect, and try it out!

keybindings:
P: draw plane on/off
F: fit plane on/off
S: save a HD color image from the camera to sketch folder
+/-: change point cloud display density (reduce density if framerate is too low)
space: toggle kinect point cloud refresh

tuning the planeRANSAC function:
the function has three inputs- tolerance (max error for a point to be considered part of consensus set), 
                               threshold (minimum percentage of points that need to be in the consensus set),
                               iterations (how many iterations to run the program for)
To tune, start with a relatively large tolerance and low threshold.
Decrease tolerance / increase threshold until your plane jumps around less but still fits correctly in the set number of iterations.
Lower iterations will mean the function runs faster, but will be a bit less accurate.
*/
 
import KinectPV2.*;
import peasy.*;
import java.nio.*;
import java.text.DecimalFormat;

KinectPV2 kinect;
PeasyCam cam;

DecimalFormat df2 = new DecimalFormat("#0.00"); //two decimal point format
DecimalFormat df3 = new DecimalFormat("#0.000"); //three decimal point format

ArrayList <PVector> pointCloud = new ArrayList <PVector>(); //point cloud
PVector[] bestPlane = new PVector[2]; //best plane
boolean planeFitInProgress = false; //whether there is a plane fit thread running already (essentially a lock)

boolean debug = true; //toggle debug messages
boolean fitPlane = false; //toggle plane
boolean drawPlane = false; //toggle plane
boolean refresh = true; //toggle refresh
float cloudDensity = 0.7; //percentage of cloud points to draw

//wrapper function to fit plane; runs in separate thread
void fitPlane() {
  if (!planeFitInProgress) {
    planeFitInProgress = true; //lock
    bestPlane = planeRANSAC(pointCloud, 1.0, 0.23, 70); //fit the plane
    planeFitInProgress = false; //unlock
  }
}

void setup() {
  //set up kinect
  kinect = new KinectPV2(this);
  kinect.enableColorImg(true);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);
  kinect.init();

  size(1920, 1080, P3D);
  cam = new PeasyCam(this, 100);
  cam.rotateY(radians(180)); //rotate so we look at Kinect point cloud by default
  frameRate(60);
  
  delay(1000); //delay 1 second so that kinect has time to initialize
}

void draw() {
  scale(1, -1); //flips display for right-hand rule, more common in robotics
  background(0); //clear background

  cam.beginHUD(); //begin heads-up display
  textSize(20);
  fill(192, 192, 0);
  text("FPS " + df2.format(frameRate), 0, 20); //displays framerate
  //displays keybindings
  text("P  | plane display " + drawPlane, 5, 40);
  text("+/-  | cloud density " + round(cloudDensity*100) + "%", 5, 60);
  text("1  | camera side view", 5, 80);
  text("spacebar  | point cloud refresh " + refresh, 5, 100);
  text("F  | plane fitting " + fitPlane, 5, 120);
  text("S  | save camera image", 5, 140);
  cam.endHUD(); //end heads-up display

  drawAxes(20); //draw centered axes

  if (refresh) pointCloud = getPointCloud(); //get the pointCloud
  int dM = int(map(cloudDensity, 0.1, 1, 10, 1)); //scale density modifier
  //draw the pointCloud
  stroke(255);
  for (int i = 0; i < pointCloud.size(); i++) {
    if (i%dM == 0) { //draw points by density modifier
      PVector p = pointCloud.get(i); //get this point
      point(p.x, p.y, p.z);//draw translated point
    }
  }

  if (fitPlane && !planeFitInProgress) thread("fitPlane"); //fit plane in separate thread

  if (drawPlane) {
    try {
      PVector pC = bestPlane[0]; //plane center
      PVector pN = bestPlane[1]; //plane normal vector
      PVector pNPoint1 = PVector.add(pC, PVector.mult(pN.normalize(), 100));
      PVector pNPoint2 = PVector.sub(pC, PVector.mult(pN.normalize(), 100));
      stroke(255);
      strokeWeight(10);
      point(pC.x, pC.y, pC.z); //draw plane center point
      stroke(0, 0, 192);
      strokeWeight(3);
      line(pNPoint1.x, pNPoint1.y, pNPoint1.z, pNPoint2.x, pNPoint2.y, pNPoint2.z); //draw plane normal vector
      PVector[] corners = getPlaneCorners(bestPlane, 600, 400); //calculate the plane corners
      PVector tl = corners[0];
      PVector bl = corners[1];
      PVector br = corners[2];
      PVector tr = corners[3];
      //draw plane
      stroke(255);
      fill(0, 192, 0);
      strokeWeight(1);
      beginShape();
      vertex(tl.x, tl.y, tl.z);
      vertex(bl.x, bl.y, bl.z);
      vertex(br.x, br.y, br.z);
      vertex(tr.x, tr.y, tr.z);
      endShape(CLOSE);
    }
    catch (IndexOutOfBoundsException e) {
      if (debug) println("ERROR: Index out of bounds, point cloud probably empty");
    }
    catch (NullPointerException n) {
      if (debug) println("ERROR: no plane");
    }
  }
}

public void keyReleased() {
  if (key == 'p') drawPlane = !drawPlane;
  if (key == 'f') fitPlane = !fitPlane;
  if (key == 's') getAndSaveImg();
  if (key == '+' || key == '=') {
    cloudDensity += 0.1;
    if (cloudDensity > 1.0) cloudDensity = 1.0;
  }
  if (key == '-') {
    cloudDensity -= 0.1;
    if (cloudDensity < 0.1) cloudDensity = 0.1;
  }
  if (key == ' ') refresh = !refresh;
}
