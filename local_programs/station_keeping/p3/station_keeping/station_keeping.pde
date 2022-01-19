Polar craftPos;
Polar craftVel;
Polar goalPos;
Polar goalVel;

int state; // 0 -> still, 1 -> rotating, 2 -> moving, 3 -> orienting.

float prevDistGoal = 1000;
float distThresh = 2;
float stillThresh = 5;
float dirThresh = 0.05;

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
  
  float dirToGoal = normalisePsi(craftVel.psi - (goalPos.minus(craftPos)).psi);
  float dirToOrientation = normalisePsi(craftVel.psi - goalVel.psi);
  float distToGoal = goalPos.minus(craftPos).mag;
   
  if (state == 0){ // still
    print("still");
    if (abs(distToGoal) > stillThresh) {
      state = 1;
    }
  }
  
  if (state == 1) {            // rotating
    print("rotating ... ");
    if (abs(dirToGoal) < dirThresh){
      print("going to moving state ...");
      state = 2;
    } else {
      if (dirToGoal > 0){
        craftVel = new Polar(craftVel.psi - 0.01, craftVel.mag);
      } else {
        craftVel = new Polar(craftVel.psi + 0.01, craftVel.mag);
      }
    }
  } 
  
  float craftThrust = 0f;
  if (state == 2) {     // moving
    if (distToGoal > prevDistGoal) {
      state = 1;
    } else if (abs(distToGoal) < distThresh) {
      state = 3;
    } else {
      craftThrust = 0.0001*distToGoal;
    }
  }
  
  if (state == 3){
    if (abs(dirToOrientation) < dirThresh){
      state = 0;
    } else {
      if (dirToOrientation > 0){
        craftVel = new Polar(craftVel.psi - 0.01, craftVel.mag);
      } else {
        craftVel = new Polar(craftVel.psi + 0.01, craftVel.mag);
      }
    }
  }
      
  prevDistGoal = distToGoal;


  //goal
  posAndVel(goalPos, goalVel, 0);

  // craft
  posAndVel(craftPos, craftVel, craftThrust);
  
  // debug
  fill(0);
  text(state, 10,12);
  text(distToGoal, 10, 24);
  text(dirToGoal, 10, 36);
  text(dirToOrientation, 10, 48);

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
    goalVel.psi = PI/2;
  if (keyCode == LEFT)
    goalVel.psi = PI;
  if (keyCode == RIGHT)
    goalVel.psi = 0;
  if (keyCode == DOWN)
    goalVel.psi = 3*(PI/2);
}
