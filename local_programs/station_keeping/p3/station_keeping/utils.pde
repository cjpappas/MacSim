void pline(Polar start, Polar end) {
  line(start.getX(), start.getY(), end.getX(), end.getY());
}

void pcircle(Polar center, float size) {
  circle(center.getX(), center.getY(), size);
}

void posAndVel(Polar pos, Polar vel, float thrust) {
  noFill();
  strokeWeight(1);
  stroke(0);
  pcircle(pos, 20);
  // black for the indication
  stroke(0);
  strokeWeight(1);
  pline(pos, pos.plus(new Polar(vel.theta, 20)));
  // mag part in green
  stroke(0, 180, 0);
  strokeWeight(2);
  pline(pos, pos.plus(vel.mult(100)));
  //trust
  stroke(180, 0, 0);
  strokeWeight(3);
  pline(pos, pos.plus(new Polar(vel.theta+ PI, thrust*1000)));
}

Polar fromCartesian(float x, float y){
  return new Polar(atan2(y,x), sqrt(x*x+y*y));
}

float normaliseTheta(float theta){
   if (theta > TWO_PI){
      return theta - TWO_PI;
    }
    if (theta < 0){
      return theta + TWO_PI;
    }
    return theta;
}
