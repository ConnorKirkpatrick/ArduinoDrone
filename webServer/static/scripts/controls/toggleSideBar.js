/**
 * function used to hide and show the sidebar, expands and shrinks the map area in relation
 */
function toggleSideBar(){
    let mapArea = document.getElementById("mapArea")
    let dock = document.getElementById("sidebar")
    if(sideBarToggle){
        sideBarToggle = 0
        dock.hidden = true
        mapArea.setAttribute("style", "width: 100%")
    }
    else{
        sideBarToggle = 1
        dock.hidden = false
        mapArea.setAttribute("style", "width: 70%")
    }
}