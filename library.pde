//for getAndSaveImg
PImage frame;
int frameIndex = 0;
//for getPointCloud
ArrayList <PVector> savedPoints = new ArrayList<PVector>();

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
