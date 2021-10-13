/*
Api needs to be able to control the boat:
- Thrusters -> increase/decrease or set?
- Rudders -> increase/decrease of set?

can hack into rudder/thruster controls
*/

/*
  Set thruster value to provided value.
  Constrained between -2 and 2
*/
function setThruster(value){
    if(value > 2) value = 2;
    else if (value < -2) value =-2;
    document.getElementById("thruster").value = value;
}
function thrusterOff(){
    document.getElementById("thruster").value = 0;
}

/*
  Set rudder value to provided value.
  Constrained between -0.7 and 0.7 
*/
function setRudder(value){
    if(value > 0.7) value = 0.7;
    else if (value < -0.7) value = -0.7;
    document.getElementById("rudder").value = value;
}
function rudderNeutral(){
    document.getElementById("rudder").value = 0;
}