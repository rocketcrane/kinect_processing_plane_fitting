/*
LICENSE & COPYRIGHT:
Copyright© 2022, Lingxiu C Zhang (https://github.com/rocketcrane/kinect_processing_plane_fitting)
 
Released under GPL-3.0-or-later
This file is part of the plane_fitting_test program. (v1.0)
Plane_fitting_test is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

/*
This is a simple plane-fitting program on a generated point cloud, using a RANSAC-like algorithm. For the kinect version, look at kinect_plane_fitting.
 
keybindings:
P: draw plane on/off
+/-: change point cloud display density (reduce density if framerate is too low)

tuning the planeRANSAC function:
the function has three inputs- tolerance (max error for a point to be considered part of consensus set),
threshold (minimum percentage of points that need to be in the consensus set),
iterations (how many iterations to run the program for)
To tune, start with a relatively large tolerance and low threshold.
Decrease tolerance / increase threshold until your plane jumps around less but still fits correctly in the set number of iterations.
Lower iterations will mean the function runs faster, but will be a bit less accurate.
*/

import peasy.*;
import java.nio.*;
import java.text.DecimalFormat;

PeasyCam cam;

DecimalFormat df3 = new DecimalFormat("#0.000"); //three decimal point format

PVector[] plane = new PVector[2];
ArrayList <PVector> pointCloud = new ArrayList <PVector>(); //point cloud
boolean planeFitInProgress = false; //whether there is a plane fit thread running already (essentially a lock)

boolean debug = true; //toggle debug
boolean drawPlane = false; //toggle plane
float cloudDensity = 1.0; //percentage of cloud points to draw

//properties of point cloud
int inliers = 1000;
int outliers = 100;

//wrapper function to fit plane; runs in separate thread
void fitPlane() {
  if (!planeFitInProgress) {
    planeFitInProgress = true; //lock
    plane = planeRANSAC(pointCloud, 0.6, 0.4, 70); //fit the plane
    planeFitInProgress = false; //unlock
  }
}

void setup() {
  size(1920, 1080, P3D);
  cam = new PeasyCam(this, 100);
  frameRate(60);

  for (int i = 0; i < inliers; i++) {
    pointCloud.add(new PVector(random(-100, 100), random(-100, 100), random(-1, 1)));
  }
  for (int i = 0; i < outliers; i++) {
    pointCloud.add(new PVector(0, 0, random(10, 20)));
  }
}

void draw() {
  scale(1, -1); //right-hand rule
  background(0); //clears background

  cam.beginHUD(); //begin heads-up display
  textSize(20);
  fill(192, 192, 0);
  text(frameRate, 0, 20); //displays framerate
  //displays keybindings
  text("P - plane " + drawPlane, 5, 40);
  text("+/- - cloud density " + round(cloudDensity*100) + "%", 5, 60);
  cam.endHUD(); //end heads-up display
  //draw centered axes
  drawAxes(20);

  int dM = int(map(cloudDensity, 0.1, 1, 10, 1)); //scale density modifier
  //draw the pointCloud
  stroke(255);
  for (int i = 0; i < pointCloud.size(); i++) {
    if (i%dM == 0) { //draw points by density modifier
      PVector p = pointCloud.get(i); //get this point
      point(p.x, p.y, p.z);//draw translated point
    }
  }

  if (!planeFitInProgress) thread("fitPlane"); //fit plane in separate thread

  if (drawPlane) {
    try {
      PVector pC = plane[0];
      PVector pN = plane[1];
      PVector pNPoint1 = PVector.add(pC, PVector.mult(pN.normalize(), 100));
      PVector pNPoint2 = PVector.sub(pC, PVector.mult(pN.normalize(), 100));
      stroke(255);
      strokeWeight(10);
      point(pC.x, pC.y, pC.z); //draw plane center point
      stroke(0, 0, 192);
      strokeWeight(3);
      line(pNPoint1.x, pNPoint1.y, pNPoint1.z, pNPoint2.x, pNPoint2.y, pNPoint2.z); //draw plane normal vector
      PVector[] corners = getPlaneCorners(plane, 600, 400); //calculate the plane corners
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

      if (debug) println("plane center = ", plane[0], " normal vector = ", plane[1], "\n"); //DEBUG
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
  if (key == '+' || key == '=') {
    cloudDensity += 0.1;
    if (cloudDensity > 1.0) cloudDensity = 1.0;
  }
  if (key == '-') {
    cloudDensity -= 0.1;
    if (cloudDensity < 0.1) cloudDensity = 0.1;
  }
}
