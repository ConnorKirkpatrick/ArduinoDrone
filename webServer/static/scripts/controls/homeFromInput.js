function homeFromInput(){
    let coordinates = document.getElementById("newHome").value

    coordinates = checkData(coordinates)
    if(!coordinates){
        alert("Incorrect coordinate type")
        return
    }
    else{
        let Lat = coordinates[0]
        let Long = coordinates[1]
        addMarker(Long,Lat,0)
        alterMarkerStyle(0,10,'#3b245d')
        document.getElementById("newHome").value = ""
    }

}