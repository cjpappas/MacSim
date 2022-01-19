const { init, startSim } = require("./api.js");
console.log(init);

const craft = init("localhost");

class Polar{
  constructor(psi, mag){
    this.psi = psi;
    this.mag = mag;
  }

  plus(other){
    return new Polar( this.psi + atan2(other.mag*sin(other.psi - this.psi), this.mag + other.mag*cos(other.psi-this.psi))
    , sqrt(other.mag*other.mag + this.mag*this.mag + 2*this.mag*other.mag*cos(other.psi-this.psi))
    );
  }

  mult(other){
    return new Polar(this.psi, this.mag*amt);
  }

  minus(other){
    return this.plus(other.mult(-1));
  }
}

let craftPos = new Polar(0,10);

function draw(){
  craft.log(craft.getPosition());
  // craft.log(craft.getVelocity());
  craft.moveForward(1);

}

let inter = setInterval(draw, 1000);

setTimeout(() => {clearInterval(inter), exit();}, 50000);