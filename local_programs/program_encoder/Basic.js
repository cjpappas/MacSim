// Methods are provided on a global craft object
craft.moveForward(1)

// They return promises and can be chained together
craft.moveForward(1)
  .then(() => craft.rotateAnticlockwise(1))
  .catch((err) => console.log("Something went wrong: " + err));

// We also provide "immediate" modes of controlling the boat and provide finer-grained control over the craft.
craft.setLeftThrusterPower(10);
craft.setRightThrusterPower(10);

// Various bits of information about the simulation can be found by calling any of the "getters" in the api
console.log(craft.getPosition());

/* 
  You can use the setInterval function to run an "execution loop" to provide continuous control over the simulation
  This is similar to the draw loop in processing - it's also a good idea to stop this when the simulation stops.
  This example shows you how you can combine various api calls into an algorithm that runs every 2 seconds.
*/
var interval = setInterval(() => {
  if(craft.getPosition().lat < craft.getGoalPosition().lat){
    craft.moveForward(1)
      .then(() => {
        if(craft.getPosition().heading < craft.getGoalPosition().heading){
          craft.rotateAnticlockwise(1);
        } else {
          craft.stop();
        }})
      .catch((err) => console.log("Something went wrong: " + err));
  }
  if(craft.getTaskInfo().state === "finished"){
    clearInterval(interval);
  }
}, 2000);