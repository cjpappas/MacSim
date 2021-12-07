class Polar {
  float theta;
  float mag;

  // you can't create a malformed Polar, if you give values outside the bounds, you get adjust back into bounds
  // this, combined with the fact that all operations create new Polars, keeps us in line.
  Polar(float theta, float mag) {
    this.theta = theta;
    this.mag = mag;
    if (this.mag < 0){
      this.theta = (this.theta - PI);
      this.mag = abs(this.mag);
    }
    this.theta = normaliseTheta(this.theta);
  }

  PVector toCartesian() {
    return new PVector(getX(), getY());
  }

  float getX() {
    return width/2 + mag*cos(theta);
  }

  float getY() {
    return height/2 - mag*sin(theta);
  }

  Polar plus(Polar other) {
    return new Polar( this.theta + atan2(other.mag*sin(other.theta - this.theta), this.mag + other.mag*cos(other.theta-this.theta))
      , sqrt(other.mag*other.mag + this.mag*this.mag + 2*this.mag*other.mag*cos(other.theta-this.theta))
      );
  }
  
  Polar minus(Polar other) {
    return this.plus(other.mult(-1));
  }

  Polar mult(float amt) {
    return new Polar(this.theta, this.mag*amt);
  }
}
