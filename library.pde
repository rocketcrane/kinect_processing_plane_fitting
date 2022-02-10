//for getAndSaveImg
PImage frame;
int frameIndex = 0;
//for getPointCloud
ArrayList <PVector> savedPoints = new ArrayList<PVector>();
//for RANSAC
PVector[] plane = new PVector[2]; //first item is point on plane, second item is normal vector

void getAndSaveImg() {
  frame = kinect.getColorImage();
  frame.save("kinect"+str(frameIndex));
  frameIndex++;
}

void getPointCloud() {
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
}

void RANSAC() {
  PVector[] currentPoints = new PVector[3]; //stores points for current iteration
  
  for(int i =0; i < 3; i++) { //iterate to get 3 points so we can fit a plane
    PVector point = savedPoints.get(int(random(1,savedPoints.size()))); //gets a random point
    currentPoints[i] = point; //saves point
  }
  
  //now let's compute the plane from these three points
  PVector vec1 = PVector.sub(currentPoints[1], currentPoints[0]); //vector from 0 to 1
  PVector vec2 = PVector.sub(currentPoints[2], currentPoints[0]); //vector from 0 to 2
  PVector vec3 = vec1.cross(vec2); //normal vector to plane
  plane[0] = currentPoints[0]; //point on plane (first item)
  plane[1] = vec3; //normal vector (second item)
  
  //now let's determine how many points from all the points fit within tolerance
}
