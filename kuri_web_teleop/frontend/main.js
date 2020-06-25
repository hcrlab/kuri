import { connectToROS, publishTopic } from "./src/utils.js";
import { Interface } from "./src/keyboard.js";
import { ScanInterface } from "./src/scan.js";

const ros = connectToROS();

// Listen for keypresses
const keyboardInterface = new Interface(document, ros);

keyboardInterface.watch()

// Draw the lidar readings bar
const scanDiv = document.getElementById("scancontainer");

const scanInterface = new ScanInterface(scanDiv, ros);