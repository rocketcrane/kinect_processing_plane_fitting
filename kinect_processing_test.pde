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
DecimalFormat df = new DecimalFormat("#0.00"); //two decimal point format

boolean drawPlane = false; //toggle plane
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
  frameRate(30);
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
  text("P - draw plane " + drawPlane, 5, 40);
  text("+/- - cloud density " + round(cloudDensity*100) + "%", 5, 60);
  cam.endHUD(); //end heads-up display

  //draw centered axes
  drawAxes(20);

  ArrayList <PVector> pointCloud = getPointCloud(); //get the pointCloud
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
      PVector[] bestPlane = planeRANSAC(pointCloud, 1.0, 0.8, 30); //fit the plane
      //float[] angles = rotationAngles(bestPlane[1]); //find the rotation angles
      //draw plane
      stroke(255);
      strokeWeight(10);
      point(bestPlane[0].x, bestPlane[0].y, bestPlane[0].z); //draw plane center point
      PVector[] corners = getPlaneCorners(bestPlane, 600, 400);
      PVector tl = corners[0]; PVector bl = corners[1]; PVector br = corners[2]; PVector tr = corners[3];
      fill(0,192,0);
      strokeWeight(1);
      beginShape();
      vertex(tl.x,tl.y,tl.z);
      vertex(bl.x,bl.y,bl.z);
      vertex(br.x,br.y,br.z);
      vertex(tr.x,tr.y,tr.z);
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
}
