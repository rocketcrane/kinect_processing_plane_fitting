/*
Test goals:
 1. capture color image
 2. capture point cloud and get an averaged plane from it
 */
import KinectPV2.*;
import peasy.*;
import java.nio.*;
import java.text.DecimalFormat;

KinectPV2 kinect;
PeasyCam cam;
CameraState state;
ArrayList <PVector> pointCloud = new ArrayList <PVector>(); //point cloud
PVector[] bestPlane = new PVector[2]; //best plane
DecimalFormat df = new DecimalFormat("#0.00"); //two decimal point format

boolean drawPlane = false; //toggle plane
boolean refresh = true; //toggle refresh
float cloudDensity = 0.5; //percentage of cloud points to draw

void setup() {
  kinect = new KinectPV2(this);
  kinect.enableColorImg(true);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);

  kinect.init();

  size(1920, 1080, P3D);
  cam = new PeasyCam(this, 100);
  cam.rotateY(radians(180)); //rotate so we look at point cloud by default
  frameRate(60);
  delay(1000); //delay so that kinect has time to initialize
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
  text("1 - camera side view", 5, 80);
  text("spacebar - refresh " + refresh, 5, 100);
  cam.endHUD(); //end heads-up display

  //draw centered axes
  drawAxes(20);

  if (refresh) pointCloud = getPointCloud(); //get the pointCloud
  int dM = int(map(cloudDensity, 0.1, 1, 10, 1)); //scale density modifier
  //draw the pointCloud
  stroke(255);
  for (int i = 0; i < pointCloud.size(); i++) {
    if (i%dM == 0) { //draw points by density modifier
      PVector p = pointCloud.get(i); //get this point
      point(p.x, p.y, p.z);//draw translated point
      //translate(p.x, p.y, p.z); //can also use translate to draw objects, but much more graphics intensive
    }
  }

  if (drawPlane) {
    try {
      if (refresh) bestPlane = planeRANSAC(pointCloud, 0.1, 0.9, 100); //fit the plane
      PVector pC = bestPlane[0];
      PVector pN = bestPlane[1];
      PVector pNPoint1 = PVector.add(pC, PVector.mult(pN.normalize(), 100));
      PVector pNPoint2 = PVector.sub(pC, PVector.mult(pN.normalize(), 100));
      //float[] angles = rotationAngles(bestPlane[1]); //find the rotation angles
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
      println("ERROR: Index out of bounds, point cloud probably empty");
    }
    catch (NullPointerException n) {
      println("ERROR: planeRANSAC did not successfully fit plane");
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
  if (key == '1') {
    try {
      cam.setState(state, 1000);
    }
    catch (NullPointerException n) {
      println("ERROR: camera state not yet set");
    }
  }
  if (key == ' ') refresh = !refresh;
}
