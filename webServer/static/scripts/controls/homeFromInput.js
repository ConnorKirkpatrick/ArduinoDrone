function homeFromInput(){
    let coords = document.getElementById("newHome").value

    //TODO breakout later
    let splitCoords = coords.split(",")
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
    addMarker(Long,Lat,0)
    alterMarkerStyle(0,10,'#3b245d')
    document.getElementById("newHome").value = ""
}