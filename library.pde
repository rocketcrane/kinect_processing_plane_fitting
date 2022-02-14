//function to draw 3D-axes
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

//function to calculate average value of an ArrayList
float getArrayListAvg(ArrayList l) {
  double total = 0.0;
  for (int i = 0; i < l.size(); i++) {
    total += Double.valueOf(l.get(i).toString());
  }
  float avg = (float) total / l.size();
  return (avg);
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
  //println(savedPoints); //DEBUG, print all points
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
  PVector[] corners = {tl, bl, br, tr};
  return (corners);
}

//DEBUG
ArrayList <Integer> bestIterations = new ArrayList<Integer>();
ArrayList <Float> bpConsensusPercentages = new ArrayList<Float>();
ArrayList <Float> minAvgErrs = new ArrayList<Float>();

//RANSAC function to get best fit plane
PVector[] planeRANSAC(ArrayList <PVector> pointCloud, float tolerance, float thresholdPercentage, int iterations) {
  PVector[] bestPlane = new PVector[2]; //to store best plane
  float minAvgErr = Float.POSITIVE_INFINITY; //to store minimum average of errors of consensus points
  int pointCloudSize = pointCloud.size(); //point cloud size
  int threshold = round(thresholdPercentage * pointCloudSize);  //the threshold number of points

  //DEBUG
  float bpConsensusPercentage = 0.0; //percentage consensus for best plane
  int bestIteration = 0; //best iteration

  //iteratively run the RANSAC
  for (int j = 0; j < iterations; j++) {
    PVector[] plane = new PVector[2]; //to store current plane
    PVector[] points = new PVector[3]; //to store 3 random points
    ArrayList <PVector> consensusPoints = new ArrayList<PVector>(); //to store the consensus points
    PVector consensusSum = new PVector(0, 0, 0); //to store vector sum of all consensus points
    float errSum = 0.0; //to store sum of errors of consensus points

    //let's first get 3 random points
    for (int i = 0; i < 3; i++) {
      PVector p = pointCloud.get(int(random(1, pointCloudSize))); //get random point

      //check that they're not the same
      if (i == 1) {
        //check that 2nd point is different from 1st
        while (p.equals(points[0])) {
          p = pointCloud.get(int(random(1, pointCloudSize)));
        }
      } else if (i == 2) {
        //check that 3rd point is different from 1 and 2
        while (p.equals(points[0]) || p.equals(points[1])) {
          p = pointCloud.get(int(random(1, pointCloudSize)));
        }
      } //end check

      points[i] = p; //save p to points
    } //end get 3 points

    //now let's compute the plane from 3 points
    PVector p0p1 = PVector.sub(points[1], points[0]); //vector from p0 to p1
    PVector p0p2 = PVector.sub(points[2], points[0]); //vector from p0 to p2
    PVector norm = p0p1.cross(p0p2).normalize(); //plane normal vector
    plane[0] = points[0]; //save p0 as plane point
    plane[1] = norm; //save normal vector
    //if  normal vector is 0 then start over with new random points (skip to next iteration)
    if (plane[1].equals(new PVector(0, 0, 0))) {
      //println("ERROR: normal vector zero, starting over with new random points..."); //DEBUG
      continue;
    }

    //now let's determine which points from the pointCLoud are consensus points
    for (int i = 0; i < pointCloudSize; i++) {
      PVector p = pointCloud.get(i); //get point

      //distance from point to plane
      PVector p0p = PVector.sub(p, plane[0]); //vector from p0 to point
      float err = abs(p0p.mag() * cos(PVector.angleBetween(p0p, plane[1]))); //err = |d|, d = |p0p|cos(a), a = angle btwn p0p and normal vector

      //if consensus point, save
      if (err <= tolerance) {
        consensusPoints.add(p);
        consensusSum.add(p);
        errSum += err;
      }
    } //end find consensus points

    //now let's decide whether this iteration has produced a better fit plane than past iterations
    int numConsensusPoints = consensusPoints.size(); //number of consensus points
    float avgErr = errSum/numConsensusPoints;
    if (avgErr < minAvgErr && numConsensusPoints > threshold) {
      plane[0] = consensusSum.div(float(numConsensusPoints)); //new plane center is average of all consensus points
      minAvgErr = avgErr; //update min sum of errors
      bestPlane = plane; //update best plane
      
      if (debug) { //DEBUG
        bestIteration = j; //update best iteration
        bpConsensusPercentage =  Float.valueOf(df3.format(float(numConsensusPoints)/float(pointCloudSize))); //update consensus percentage best plane
        print("  minAvgErr = ", df3.format(minAvgErr));
        print("  bpConsensusPercentage = ", df3.format(bpConsensusPercentage));
        println("  iterations = ", j);
      }
    }
  } //end single iteration
  
  //DEBUG
  if (debug) {
    bestIterations.add(bestIteration); //this run's best iteration number
    bpConsensusPercentages.add(bpConsensusPercentage); //this run's best plane consensus percentage
    minAvgErrs.add(minAvgErr); //this run's minimum average error
    print("avg min error = ", df3.format(getArrayListAvg(minAvgErrs)), "  avg best iteration = ", round(getArrayListAvg(bestIterations)));
    println("  avg best plane consensus percentage = ", df3.format(getArrayListAvg(bpConsensusPercentages)), " (inclusive of all RANSAC runs)\n");
  }
  
  //if unsuccessful, print error message
  if (minAvgErr == Float.POSITIVE_INFINITY && debug) println("ERROR: planeRANSAC did not successfully fit plane");
  return (bestPlane);
}
