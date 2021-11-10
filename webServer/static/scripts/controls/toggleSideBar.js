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