class Polar {
  float psi;
  float mag;

  // you can't create a malformed Polar, if you give values outside the bounds, you get adjust back into bounds
  // this, combined with the fact that all operations create new Polars, keeps us in line.
  Polar(float psi, float mag) {
    this.psi = psi;
    this.mag = mag;
    if (this.mag < 0){
      this.psi = (this.psi - PI);
      this.mag = abs(this.mag);
    }
    this.psi = normalisePsi(this.psi);
  }

  PVector toCartesian() {
    return new PVector(getX(), getY());
  }

  float getX() {
    return width/2 + mag*cos(psi);
  }

  float getY() {
    return height/2 - mag*sin(psi);
  }

  Polar plus(Polar other) {
    return new Polar( this.psi + atan2(other.mag*sin(other.psi - this.psi), this.mag + other.mag*cos(other.psi-this.psi))
      , sqrt(other.mag*other.mag + this.mag*this.mag + 2*this.mag*other.mag*cos(other.psi-this.psi))
      );
  }
  
  Polar minus(Polar other) {
    return this.plus(other.mult(-1));
  }

  Polar mult(float amt) {
    return new Polar(this.psi, this.mag*amt);
  }
}
