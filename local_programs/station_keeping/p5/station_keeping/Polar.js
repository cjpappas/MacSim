class Polar {
  /* radius is first param, theta is second
     
     Our polar co-ordinates _won't_ have negatives.  r will range from 0 to inf, theta will range
     from 0 to 2*pi.  The constructor is responsible for "normalising" any out of range parameters.
     
     Don't adjust an existing polar, always create new ones, so the normalisation will occur.
  */
  constructor(r, theta){
    let normaliseTheta = (theta) => {
       if (theta > TWO_PI){
          return theta - TWO_PI;
        }
        if (theta < 0){
          return theta + TWO_PI;
        }
        return theta;
      };
    this.r = r;
    this.theta = theta;
    if(this.r < 0){
      this.theta = this.theta - PI;
      this.r = abs(this.r);
    }
    this.theta = normaliseTheta(this.theta);
  }
  
  plus(other){
    return new Polar(this.theta + atan2(other.r*sin(other.theta - this.theta)
                                       ,this.r + other.r*cos(other.theta - this.theta)
                                       )
                    , sqrt(other.r*other.r + this.r*this.r + 2*this.r*other.r*cos(other.theta - this.theta))
                    );
  }
  
  mult(scale){
    return new Polar(this.r*scale, this.theta);
  }
  
  minus(other){
    return this.plus(other.mult(-1));
  }
  
  
}
