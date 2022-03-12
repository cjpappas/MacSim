const { init } = require("./api");

const craft = init(process.env.IP);

while (true){
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