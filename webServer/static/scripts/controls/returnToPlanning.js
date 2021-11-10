function returnToPlanning(){
    //delete the lines
    let len = lineArray.length
    for(let i = 0; i < len; i++){
        removeLine(lineArray[0].getId())
    }
    //enable the buttons
    document.getElementById("newWPNumber").disabled = false
    document.getElementById("newWP").disabled = false
    document.getElementById("newWPButton").disabled = false

    document.getElementById("newHome").disabled = false
    document.getElementById("newHomeButton").disabled = false

    document.getElementById("createFlight").disabled = false
    let buttons = document.getElementsByClassName("removeButton")
    for(let i = 0; i < buttons.length; i++){
        buttons[i].disabled = false
    }

    //disable buttons to start flight or go back to planning
    document.getElementById("startFlight").disabled = true
    document.getElementById("returnToPlanning").disabled = true
}
