ROSLIB = require("roslib");

const ws_url = `ws://0.0.0.0:9090`;

console.log(`attempting to connect on ${ws_url}`)
connection = new ROSLIB.Ros({ url: ws_url});
connection.on("connection", () => {console.log("Connected to rosbridge server!"); setup_connection()});
connection.on("error", () => console.log("Error connecting to rosbridge"));
connection.on("close", () => console.log("Connection to rosbridge server closed."));

function setup_connection(){
  info = new ROSLIB.Topic({
    ros:connection,
    name: "/vrx/task/info",
    messageType: "vrx_gazebo/Task"
  });
  info.subscribe((message)=>{
    console.log('Received message on ' + listener.name + ': ' + message.data);
  })
}
while (false){
  if(craft.data.task.state === "running"){
    switch(craft.data.task.name){
      case "station_keeping":   while_running(station_keeping, craft);   break;
      case "wayfinding":        while_running(wayfinding, craft);        break;
      case "perception":        while_running(perception, craft);        break;
      case "wildlife":          while_running(wildlife, craft);          break;
      case "gymkhana":          while_running(gymkhana, craft);          break;
      case "scan_dock_deliver": while_running(scan_dock_deliver, craft); break;
      
    }
  }
}

const while_running = (fn, craft) => {
  console.log(fn.name + " has been summoned");
  while(craft.data.task.state === running){
    fn(craft);
  }
}

const station_keeping = (craft) => {
  console.log("inspecting");
  console.log("deciding");
}

const wayfinding = (craft) => {
  console.log("inspecting");
  console.log("deciding");
}

const perception = (craft) => {
  console.log("inspecting");
  console.log("deciding");
}

const wildlife = (craft) => {
  console.log("inspecting");
  console.log("deciding");
}

const gymkhana = (craft) => {
  console.log("inspecting");
  console.log("deciding");
}

const scan_dock_deliver = (craft) => {
  console.log("inspecting");
  console.log("deciding");
}