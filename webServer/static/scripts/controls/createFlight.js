function createFlight(){
    if(!home){
        alert("Home has not been set")
    }
    else {
        //make all markers some bright colour
        //create lines between every marker set: 0-1,1-2, 2-3,....
        for (let i = 0; i < markerArray.length; i++) {
            if (i === markerArray.length - 1) {
                addLine(markerArray[i], markerArray[0])
            } else {
                addLine(markerArray[i], markerArray[i + 1])
            }

        }
        //disable controls
        document.getElementById("newWPNumber").disabled = true
        document.getElementById("newWP").disabled = true
        document.getElementById("newWPButton").disabled = true

        document.getElementById("newHome").disabled = true
        document.getElementById("newHomeButton").disabled = true

        document.getElementById("createFlight").disabled = true
        let buttons = document.getElementsByClassName("removeButton")
        for (let i = 0; i < buttons.length; i++) {
            buttons[i].disabled = true
        }

        //enable buttons to start flight or go back to planning
        document.getElementById("startFlight").disabled = false
        document.getElementById("returnToPlanning").disabled = false
    }
}