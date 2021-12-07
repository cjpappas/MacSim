let vector = class {
  constructor(x, y){
    this.x = x;
    this.y = y;
  }

  plus(other){
    this.x = this.x + other.x;
    this.y = this.y + other.y;
  }

  sub(other){
    this.x = this.x - other.x;
    this.y = this.y - other.y;
  }

  mult(scale){
    this.x = this.x * scale;
    this.y = this.y * scale;
  }

  div(scale){
    this.x = this.x / scale;
    this.y = this.y / scale;
  }

  mag(){
    return Math.sqrt(this.x*this.x + this.y*this.y);
  }

  normalise(){
    if(this.mag() != 0)
      this.div(this.mag());
  }
}
