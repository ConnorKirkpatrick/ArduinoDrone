/**
 * SocketEvent newCenter
 * socket event used to pass new coordinates to the newMapCenter function to recenter the map view
 * @data{Array}, A string containing the latitude and Longitude in the form "Lat,Long"
 */
socket.on("newCenter", (data) => {
    let Long = parseFloat(data[1])
    let Lat = parseFloat(data[0])
    newMapCenter(Long, Lat)
})