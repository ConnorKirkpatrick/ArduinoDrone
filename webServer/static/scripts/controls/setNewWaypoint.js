function setNewWaypoint(){
    let id = parseInt(document.getElementById("newWPNumber").value)
    let coordinates = document.getElementById("newWP").value
    //check the data is correct
    coordinates = checkData(coordinates)
    if(!coordinates){
        alert("Incorrect coordinate type")
        return
    }
    else{
        let Lat = coordinates[0]
        let Long = coordinates[1]
        //check the ID
        if(isNaN(id)){
            alert("Ensure The ID Is A Valid Number (1-10)")
            return
        }
        else {
            //check the ID is not already in use
            for (let i = 0; i < markerArray.length; i++) {
                if (markerArray[i].getId() === id) {
                    alert("Waypoint Number Already In Use")
                    return
                }
            }
            //now create the waypoint
            addMarker(Long, Lat, id)
            //now add this to the user interface
            addInterfaceWaypoint(id, Long, Lat)
            document.getElementById("newWPNumber").value = ""
            document.getElementById("newWP").value = ""
        }
    }

}