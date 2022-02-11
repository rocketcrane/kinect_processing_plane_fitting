
void drawAxes(float size) {
  //X  - red
  stroke(192, 0, 0);
  line(0, 0, 0, size, 0, 0);
  //Y - green
  stroke(0, 192, 0);
  line(0, 0, 0, 0, size, 0);
  //Z - blue
  stroke(0, 0, 192);
  line(0, 0, 0, 0, 0, size);
}

//function to get and save an HD color image from the kinect in the sketch folder
int frameIndex = 0; //for getAndSaveImg
void getAndSaveImg() {
  PImage frame;
  frame = kinect.getColorImage();
  frame.save("kinect"+str(frameIndex));
  frameIndex++;
}

//function to get the point cloud from kinect as an arraylist of PVectors
ArrayList <PVector> getPointCloud() {
  ArrayList <PVector> savedPoints = new ArrayList <PVector> ();
  FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
  for (int i = 0; i < kinect.WIDTHDepth * kinect.HEIGHTDepth; i++) {
    float x1 = pointCloudBuffer.get(i*3 + 0)*1000;
    float y1 = pointCloudBuffer.get(i*3 + 1)*1000;
    float z1 = pointCloudBuffer.get(i*3 + 2)*1000;
    if (z1>0) {  //if our point z value is greater than zero
      PVector originalPt = new PVector(x1, y1, z1);
      savedPoints.add(originalPt);
    }
  }
  //print(savedPoints, "\n"); //DEBUG, print all points
  return (savedPoints);
}

//function to get plane corners from center point + normal vector
PVector[] getPlaneCorners(PVector[] bestPlane, float w, float h) {
  PVector y = bestPlane[1].cross(new PVector(1, 0, 0));
  PVector x = y.cross(bestPlane[1]);
  PVector hy = y.setMag(h/2);
  PVector hx = x.setMag(w/2);
  PVector tl = PVector.sub(PVector.add(bestPlane[0], hy), hx);
  PVector bl = PVector.sub(PVector.sub(bestPlane[0], hy), hx);
  PVector br = PVector.add(PVector.sub(bestPlane[0], hy), hx);
  PVector tr = PVector.add(PVector.add(bestPlane[0], hy), hx);
  PVector[] corners = {tl,bl,br,tr};
  return (corners);
}

PVector[] planeRANSAC(ArrayList <PVector> savedPoints, float tolerance, float threshold, int iterations) {
  PVector[] plane = new PVector[2]; //to store best fit plane

  //iteratively run this algorithm multiple times
  for (int j = 0; j < iterations; j++) {

    PVector[] currentPoints = new PVector[3]; //stores points for current iteration
    //let's first get three random points, the minimum required to fit a plane
    for (int i = 0; i < 3; i++) { //iterate to get 3 points
      PVector point = savedPoints.get(int(random(1, savedPoints.size()))); //gets a random point
      //check that they're not the same!
      if (i == 1) {
        //check that second point is different from first
        while (point.equals(currentPoints[0])) {
          point = savedPoints.get(int(random(1, savedPoints.size()))); //gets a new random point
        }
      } else if (i == 2) {
        //check that third point is different from first two
        while (point.equals(currentPoints[0]) || point.equals(currentPoints[1])) {
          point = savedPoints.get(int(random(1, savedPoints.size()))); //gets a new random point
        }
      } //end difference check
      currentPoints[i] = point; //saves point
    } //end for loop, we now have 3 random points

    //now let's compute the plane from these three points
    PVector vec1 = PVector.sub(currentPoints[1], currentPoints[0]).normalize(); //normalized vector from p0 to p1
    PVector vec2 = PVector.sub(currentPoints[2], currentPoints[0]).normalize(); //normalized vector from p0 to p2
    PVector vec3 = vec1.cross(vec2); //get normal vector to plane
    plane[0] = currentPoints[0]; //save point on plane (first item)
    plane[1] = vec3; //save normal vector (second item)

    //now let's determine how many points from all the points fit within tolerance
    PVector consensusSum = new PVector(0, 0, 0); //sum of all consensus points
    ArrayList <PVector> consensusPoints = new ArrayList<PVector>(); //save the consensus points
    for (int i = 0; i < savedPoints.size(); i++) { //iterate through all points
      PVector point = savedPoints.get(i); //gets current point

      //let's find the distance from the point to the plane
      PVector p0p1 = PVector.sub(point, plane[0]); //gets vector from point on plane to current point
      float dist = p0p1.mag() * cos(PVector.angleBetween(p0p1, plane[1])); //gets distance: |vector|cos(theta)

      //only save consensus points
      if (dist <= tolerance) {
        consensusPoints.add(point);
        consensusSum.add(point);
      }
    } //end for loop, we now have the consensus points

    //now, if the number of consensus points is more than the threshold, re-fit the plane with all the consensus points and exit
    if (consensusPoints.size() >= threshold * float(savedPoints.size())) {
      plane[0] = consensusSum.div(float(consensusPoints.size())); //new plane center is average of all consensus points
      println("planeRANSAC successful in ", j, " iterations, ", consensusPoints.size(), " consensus points out of ", savedPoints.size(), " total points"); //DEBUG
      return(plane);
    }
  } //end single iteration
  //println("planeRANSAC unsuccessful, returning next best solution"); //DEBUG
  return(null);
}
