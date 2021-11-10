function setNewWaypoint(){
    let id =document.getElementById("newWPNumber").value
    let coordinates = document.getElementById("newWP").value
    console.log(id, coordinates)
    //check the data is correct
    let splitCoords = coordinates.split(",")
    if(splitCoords.length !== 2){
        alert("Incorrect coordinate type")
        return
    }
    let Lat
    let Long
    try {
        Lat = parseFloat(splitCoords[0])
        Long = parseFloat(splitCoords[1])
    }
    catch (e) {
        alert("Incorrect coordinate type")
        return
    }
    //check the ID is not already in use
    for(let i = 0; i < markerArray.length; i++){
        if(markerArray[i].getId() === id){
            alert("Waypoint Number Already In Use")
            return
        }
    }
    console.log("Good Data")
    //now create the waypoint
    addMarker(Long,Lat,id)
    markerArray.sort(function (a,b){return a.getId() - b.getId()})
    for(let i = 0; i < markerArray.length; i++) {
        console.log(markerArray[i].getId())
    }
    //now add this to the user interface
    addInterfaceWaypoint(id, Long, Lat)
}