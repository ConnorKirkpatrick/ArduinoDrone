/**
 * SocketEvent newCenter
 * socket event used to pass new coordinates to the newMapCenter function to recenter the map view
 * @data{Array}, A string containing the latitude and Longitude in the form "Lat,Long"
 */
socket.on("newCenter", (data) => {
    let Long = parseFloat(data[1])
    let Lat = parseFloat(data[0])
    newMapCenter(Long, Lat)
    setTimeout(function () {
        addMarker(Long, Lat, 1)
        console.log(markerArray[0].getId())
    }, 2000)
    setTimeout(function () {
        addMarker(-0.5864,51.2411, 2)
        console.log(markerArray[0].getId())

        addLine(markerArray[0], markerArray[1])
        setTimeout(function (){
            removeLine("1-2")
        },2000)
    }, 5000)
})