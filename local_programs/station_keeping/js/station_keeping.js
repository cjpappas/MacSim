const { init, startSim } = require("./api.js");
console.log(init);

const craft = init("localhost");

function polarPlus(self, other){
  return {psi: self.psi + atan2(other.r*sin(other.psi - self.psi), self.r + other.r*cos(other.psi-self.psi))
  , r: sqrt(other.r*other.r + self.r*self.r + 2*self.r*other.r*cos(other.psi-self.psi))
  };
}

function polarMult(self, amt){
  return {psi:self.psi, r: self.r*amt};
}

function polarMinus(self, other){
  return polarPlus(self, polarMult(other, -1));
}

function draw(){
  craft.log(craft.getPosition());
  // craft.log(craft.getVelocity());
  craft.imm.setLeftThrusterPower(1);
  craft.imm.setRightThrusterPower(1);

}

let inter = setInterval(draw, 1000);

setTimeout(() => {clearInterval(inter), exit();}, 50000);

