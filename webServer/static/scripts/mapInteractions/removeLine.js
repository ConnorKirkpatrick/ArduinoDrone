/**
 * Used to remove a line from the map interface
 * @param id The ID of the line to remove
 */
function removeLine(id){
    for(let i = 0; i < lineArray.length; i++){
        if(lineArray[i].getId() === id){
            vectorSource.removeFeature(lineArray[i])
            lineArray.splice(i,1)
            break
        }
    }
}