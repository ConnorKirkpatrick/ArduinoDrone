function newMapCenter(long, lat) {
    mainView.setCenter(ol.proj.fromLonLat([long, lat]))
}