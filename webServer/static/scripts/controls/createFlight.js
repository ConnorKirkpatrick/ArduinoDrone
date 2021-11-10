function createFlight(){
    //make all markers some bright colour
    //create lines between every marker set: 0-1,1-2, 2-3,....
    for(let i = 0; i < markerArray.length; i++){
        console.log(markerArray[i].getId())
        if( i === markerArray.length-1){
            addLine(markerArray[i], markerArray[0])
        }
        else{
            addLine(markerArray[i], markerArray[i+1])
        }

    }
}