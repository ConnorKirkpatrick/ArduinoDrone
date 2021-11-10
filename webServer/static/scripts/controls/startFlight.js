function startFlight(){
    let toVisit = [...markerArray].reverse()
    toVisitLines = [...lineArray].reverse()
    let endWP = toVisit.pop()
    toVisit = [endWP,...toVisit]

    let currentWP = toVisit.pop()
    let currentLine = toVisitLines.pop()

    alterMarkerStyle(currentWP.getId(),currentMarkerSize,currentMarkerColour)
    alterLineStyle(currentLine.getId(),currentLineWidth,currentLineColour)

    for(let i = 0; i < toVisit.length; i++){
        //prevent the home marker from being altered
        if(toVisit[i].getId() !== 0){
            alterMarkerStyle(toVisit[i].getId(),baseMarkerSize,newMarkerColour)
        }

    }
    for(let i = 0; i < visitedPoints.length; i++){
        alterMarkerStyle(toVisit[i].getId(),baseMarkerSize,oldMarkerColour)
    }


    for(let i = 0; i < toVisitLines.length; i++){
        alterLineStyle(toVisitLines[i].getId(), baseLineWidth, baseLineColour)
    }
    //TODO disable planning controls
    // Hide and lock the left size bar
    // Add right side bar for the flight parameters
    // Send data via socket to be stored on the server

}