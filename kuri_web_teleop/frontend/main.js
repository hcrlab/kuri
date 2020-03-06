import { connectToROS, publishTopic } from "./src/utils.js";
import { Interface } from "./src/keyboard.js";
import { ScanInterface } from "./src/scan.js";
import { networkConfig } from "./config/network_info.js";

const ros = connectToROS(networkConfig.ip);

// Add the camera feed to DOM
let img = document.createElement("img")
img.setAttribute("class", "imagecontainer");
img.setAttribute("style", "-webkit-user-select: none;margin: auto;");
const webVideoServerUrl = "http://" + networkConfig.ip  + ":8081/stream?topic=/upward_looking_camera_overlay&amp;type=ros_compressed";
console.log(webVideoServerUrl);
img.setAttribute("src", webVideoServerUrl);

document.getElementById("camerafeed").appendChild(img);

// Listen for keypresses
const keyboardInterface = new Interface(document, ros);

keyboardInterface.watch()

// Draw the lidar readings bar
const scanDiv = document.getElementById("scancontainer");

const scanInterface = new ScanInterface(scanDiv, ros);
