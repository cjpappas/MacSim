var sinks = {};

const { WebSocketSubject } = rxjs.webSocket;
master = new WebSocketSubject("ws://localhost:9090");
master.subscribe(
  msg => {
    sinks[msg.topic](msg);
  },
  err => console.log("ERROR: " + err),
  () => console.log("COMPLETE")
);

function subscribe(topic, action) {
  sinks[topic] = action;
  master.next({op: "subscribe", topic: topic})
}

function toDegs(degrees){
  return (degrees/Math.PI)*180;
}
function invert(rads){
  return -1*rads
}
function iconFromHeading(rotation){
  return {path: "M -12 5 L -12 -5 L 0 -5 L 0 -18 L 37 0 L 0 18 L 0 5 Z", 
          fillOpacity: 1,
          fillColor: "#FFFFFF",
          scale:0.5, 
          rotation: invert(toDegs(rotation))
          }
}
function imuMsgToHeading(msg){
  var eu = new THREE.Euler();
  var ex = new THREE.Quaternion(msg.msg.orientation.x, msg.msg.orientation.y, msg.msg.orientation.z, msg.msg.orientation.w);
  eu.setFromQuaternion(ex)
  return eu.z
}
function icon(rotation){
  return {path: "M -12 5 L -12 -5 L 0 -5 L 0 -18 L 37 0 L 0 18 L 0 5 Z", 
          fillOpacity: 1,
          fillColor: "#FFFFFF",
          scale:0.5, 
          rotation:(rotation/(Math.PI))*-180
        }
}
function leftThrust(force, duration){
  master.next({op: "publish", topic: "/wamv/thrusters/left_thrust_cmd", msg: {data: force}});
  setTimeout(() => {
    master.next({op: "publish", topic: "/wamv/thrusters/left_thrust_cmd", msg: {data: 0}});
    console.log("left thrust done");
  }, duration);
}
function rightThrust(force, duration){
  master.next({op: "publish", topic: "/wamv/thrusters/right_thrust_cmd", msg: {data: force}});
  setTimeout(() => {
    master.next({op: "publish", topic: "/wamv/thrusters/right_thrust_cmd", msg: {data: 0}});
    console.log("right thrust done");
  }, duration);
}
function rotateAntiClockwise(strength){
  leftThrust(-1*strength,2000);
  rightThrust(strength,2000);
}
function rotateClockwise(strength){
  leftThrust(strength, 2000);
  rightThrust(-1*strength, 2000);
}
