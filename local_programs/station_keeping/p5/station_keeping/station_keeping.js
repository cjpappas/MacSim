let goalPos;

function setup() {
  createCanvas(800,800);
  craftPos = new Polar(0,0);
}


function draw() {
  background(255);
  stroke(0);
  strokeWeight(1);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);

  displayGoal(goalPos);
}

function displayGoal(pos){
  noFill();
  strokeWeight(1);
  stroke(0);
  pcircle(pos, 20);
}

fucntion pcircle(center, size) {
  circle(center.getX(), center.getY(), size);
}
