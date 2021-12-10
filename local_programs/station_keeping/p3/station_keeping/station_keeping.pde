Polar craftPos;
Polar craftVel;
Polar goalPos;
Polar goalVel;

int state; // 0 -> stationary, 1 -> rotating, 2 -> moving.
float prevDist = 1000;

void setup() {
  size(1200, 700);
  craftPos = new Polar(0, 0);
  craftVel = new Polar(PI/8, 0);
  goalPos = new Polar(PI/4, 300);
  goalVel = new Polar(PI, 0);
  state = 0;
}

void draw() {
  background(255);
  stroke(0);
  strokeWeight(1);
  // co-ordinates
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);

  float craftThrust = 0f;
  float dirToGoal = normaliseTheta(craftVel.theta - (goalPos.minus(craftPos)).theta);
  float dirToOrientation = normaliseTheta(craftVel.theta - goalVel.theta);
  float distToGoal = goalPos.minus(craftPos).mag;
  if (state == 0){
    if (abs(distToGoal) > 5) {
      state = 1;
    } else if (abs(dirToOrientation) > 0.05){
      state = 2;
    }
  }
  if (state == 1) {            // rotating
    print("rotating ... ");
    if (abs(distToGoal) < 5) { // lining up
      print("lining up ... ");
      if (dirToOrientation > 0.05  && dirToOrientation < 2*PI - 0.05){
        print("adjustment needed ... ");print(dirToOrientation);
        if (dirToOrientation < PI) {
          println("rotate right");
          craftVel = new Polar(craftVel.theta - 0.01, craftVel.mag);
        } else {
          println("rotate left");
          craftVel = new Polar(craftVel.theta + 0.01, craftVel.mag);
        }
      } else {
        println("close enough");
        state = 0;
      }
    } else {                   // going for goal
      print("rotating to goal ... ");
      if (abs(dirToGoal) > 0.05) {
        print("adjustment needed ... ");
        if (dirToGoal < PI) {
          println("rotate right");
          craftVel = new Polar(craftVel.theta - 0.01, craftVel.mag);
        } else {
          println("rotate left");
          craftVel = new Polar(craftVel.theta + 0.01, craftVel.mag);
        }
      } else {
        println("already pointing the right direction");
        state = 2;
      }
    }
  } else if (state == 2) {     // moving
    print("moving ... ");
    if (distToGoal > prevDist) {
      println("moving away!");
      state = 1;
    } else if (abs(distToGoal) > 5) {
      print("calculating forward or backwards ... ");
      if (distToGoal > 0) {
        println("moving forwards");
        craftThrust = 0.0001*distToGoal;
      } else {
        println("moving backwards");
        craftThrust = -0.0001*distToGoal;
      }
    } else {
      println("close enough");
      state = 1;
    }
  }
  prevDist = distToGoal;


  //goal
  posAndVel(goalPos, goalVel, 0);

  // craft
  posAndVel(craftPos, craftVel, craftThrust);

  craftVel.mag += craftThrust; // thrust
  craftVel.mag -= 0.1*craftVel.mag*craftVel.mag; // drag
  craftPos = craftPos.plus(craftVel);
  goalPos = goalPos.plus(goalVel);
}

void mousePressed(){
  goalPos = fromCartesian(mouseX-width/2, -1*(mouseY-height/2));
}

void keyPressed(){
  if (keyCode == UP)
    goalVel.theta = PI/2;
  if (keyCode == LEFT)
    goalVel.theta = PI;
  if (keyCode == RIGHT)
    goalVel.theta = 0;
  if (keyCode == DOWN)
    goalVel.theta = 3*(PI/2);
}
