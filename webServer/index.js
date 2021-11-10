const path = require("path");

const app = require("express")();
const http = require("http").createServer(app);
const io = require("socket.io")(http);
const events = require("events");
const SerialPort = require("serialport")

const serialStart = require("../functions/serialStart")
const serialWrite = require("../functions/serialWrite")
const serialRead = require("../functions/serialRead")

const express = require("express")
app.use("/static", express.static(path.join(__dirname, "./static/")));
const pug = require("pug")
app.set("view engine","pug")
app.set("views","./static/views")

const fs = require("fs")

let PORT = Number(process.env.PORT || 80);
http.listen(PORT, () => {
    console.log("Listening on " + PORT);
});

app.get("/", (req, res) => {
    res.sendFile(__dirname + "/static/webPages/mainPage/mainPage.html");
});

let port = serialStart('COM15')
serialRead(port)

port.on("open", () => {
    serialWrite(port, "This is a test")
})
io.on("connect", socket =>{
    console.log("Connection made")
})
/*
/dev/ttyS6

name is '/dev/ttyS* where * is the com number -1
 */
/*
functionality:
    user adds points along with their order into the interface -> points will be created on the map
    Create a Home point from the drones landed GPS, add to the Map as WP0, via button click, blocks below functionality
    User clicks create route, we sort the array so that the array looks like this: [0,1,2,3,4...]
        if any points are missing the process fails and pops an alert

    Once the array is sorted, create lines between each sequential point, 0-1,1-2,2-3,3-4....x-0

    New arrays created: visitedPoints, ToVisit, visitedLines, toVisitLines
    sizes and colours; line, current line, old point, current point, new point
    Set the top of ToVisit as currentWaypoint, change shape and colour to current
    Set top of toVisitLines as current Line, change shape and colour to current
    When we reach the currentwaypoint, pop it from toVisit into Visited, change to old colour
        change line to standard colour and size
    queue the next top item from both lists as the current waypoint/line

    Testing waypoints:
        Home: 51.24435086797461, -0.5858625658094214
        wp1: 51.24398128019178, -0.5952681896986041
        wp2: 51.24163472010025, -0.5945302981571259
        wp3: 51.24111128431468, -0.5864824526099666
 */