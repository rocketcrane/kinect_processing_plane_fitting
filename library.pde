
int frameIndex = 0; //for getAndSaveImg
void getAndSaveImg() {
  PImage frame;
  frame = kinect.getColorImage();
  frame.save("kinect"+str(frameIndex));
  frameIndex++;
}

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
  print(savedPoints, "\n"); //DEBUG, print all points
  return (savedPoints);
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
    ArrayList <PVector> consensusPoints = new ArrayList<PVector>(); //save the consensus points
    for (int i = 0; i < savedPoints.size(); i++) { //iterate through all points
      PVector point = savedPoints.get(i); //gets current point

      //let's find the distance from the point to the plane
      PVector p0p1 = point.sub(plane[0]); //gets vector from point on plane to current point
      float dist = p0p1.mag() * cos(PVector.angleBetween(p0p1, plane[1])); //gets distance: |vector|cos(theta)

      //only save consensus points
      if (dist <= tolerance) {
        consensusPoints.add(point);
      }
    } //end for loop, we now have the consensus points

    //now, if the number of consensus points is more than the threshold, re-fit the plane with all the consensus points and exit
    if (consensusPoints.size() >= threshold * savedPoints.size()) {
      print("planeRANSAC has found a plane within parameters\n");
      print(plane[0], plane[1]); //DEBUG, print plane parameters
      return(plane);
    }
  } //end single iteration
  print("planeRANSAC was not able to find a plane within parameters, returning best fit plane\n");
  return(plane);
}
