 // google.maps.Marker array
      var waypoints = []

      var directionsDisplay;
      var directionsService;
      var map;

      function initMap() {
        //directions views
        directionsDisplay = new google.maps.DirectionsRenderer;
        directionsService = new google.maps.DirectionsService;
        var artsQuad = new google.maps.LatLng(42.449082, -76.484529)
        map = new google.maps.Map(document.getElementById('map'), {
          zoom: 17,
          center: artsQuad
        });

        directionsDisplay.setMap(map);

        map.addListener('click', function(e) {
          var newMarker = placeMarker(e.latLng, map);
          waypoints.push(newMarker);
          //logGPS(waypoints);
          var checkedValue = document.getElementById('toggle-autoroute').checked;
          if(checkedValue){
            showRoute(directionsService, directionsDisplay);
          }
          else{
            //displayAllMarkers(map, directionsDisplay)
            //need to remove route when autoroute if off directionsDisplay.setMap(null);
          }
         });
      }

      function showRoute(directionsService, directionsDisplay){
        if(waypoints.length>1){
            calculateAndDisplayRoute(directionsService, directionsDisplay);
            for (var i = 0; i < waypoints.length; i++) {
              waypoints[i].setMap(null);
            }
          }

      }

      function editPoints(){
        if(directionsDisplay != null) {
          directionsDisplay.setMap(null);
          directionsDisplay = new google.maps.DirectionsRenderer;
          directionsDisplay.setMap(map);
        }
        displayAllMarkers(map, directionsDisplay);
      }

      function placeMarker(latLng, map) {
        var index = waypoints.length
        var marker = new google.maps.Marker({
          position: latLng,
          map: map,
          draggable: true,
          icon: 'http://chart.apis.google.com/chart?chst=d_map_pin_letter&chld='+index+'|FE6256|000000'
        });
        return marker
      }

      function reset(){
        for (var i = 0; i < waypoints.length; i++) {
          waypoints[i].setMap(null);
        }
        // clear array
        waypoints = [];
        
        if(directionsDisplay != null) {
          directionsDisplay.setMap(null);
          directionsDisplay = new google.maps.DirectionsRenderer;
          directionsDisplay.setMap(map);
        }
      }

      // test waypoints array
      function logGPS(waypoints){
        console.log("waypoint coordinates")
        for (var i = 0; i < waypoints.length; i++){
          console.log(waypoints[i].position.toString())
        }
      }


      //unused
      function calculateAndDisplayRouteBetweenTwoPoints(directionsService, directionsDisplay) {
        var selectedMode = "BICYCLING";
        var length = waypoints.length;
        var way1 = waypoints[length-2].getPosition()
        var way2 = waypoints[length-1].getPosition()
        directionsService.route({
          origin: {lat: way1.lat(), lng: way1.lng()},  
          destination: {lat: way2.lat(), lng: way2.lng()},  
          travelMode: google.maps.TravelMode[selectedMode]
        }, function(response, status) {
          if (status == 'OK') {
              directionsDisplay.setDirections(response); 
          } else {
            window.alert('Directions request failed due to ' + status);
          }
        });
      }

      function displayAllMarkers(map, directionsDisplay){
        for (var i = 0; i < waypoints.length; i++) {
          waypoints[i].setMap(map);
        }
      }

      function calculateAndDisplayRoute(directionsService, directionsDisplay) {
          var wpts = []
          for (var i = 1; i < waypoints.length-1; i++){
            wpts.push({location: waypoints[i].getPosition(), stopover:false});
          }      
          // dont zoom until 3 points have been placed
        if(waypoints.length < 3){
          directionsDisplay.setOptions({ preserveViewport: true });
        }
        directionsService.route({
          origin: waypoints[0].getPosition(),
          destination: waypoints[waypoints.length-1].getPosition(),
          //max 25 waypoints (including start and end, need to deal with this case
          waypoints: wpts,
          //optimize changes order of waypoints to shortest route
          optimizeWaypoints: false,
          travelMode: 'BICYCLING'
        }, function(response, status) {
          if (status === 'OK') {
            if (waypoints.length>1){
              directionsDisplay.setDirections(response);
            var route = response.routes[0];
            getRoutePoints(route);
            }

          } else {
            // write new error message
            window.alert('Directions request failed due to ' + status);
          }
        });
        
      }

    // returns JSON of each points along the route
    function getRoutePoints(route){
      var points = []
      var pointsJSON = {
        legs: []
      };
      
      // string encoding of route
      for(var i = 0; i < route.legs.length; i++){
        var legSteps = route.legs[i].steps
        for (var k = 0; k < legSteps.length; k++){
          var step = legSteps[k].path
          var legJSON = {
            points: []
          };

          for(var j = 0; j < step.length; j++){
            var point = step[j]
            var pointArr = [point.lat(), point.lng()]
            points.push(pointArr)
            legJSON.points.push({
              "lat": point.lat(),
              "lng": point.lng()
            }) 
          }
          pointsJSON.legs.push(legJSON)
      }

        }
        console.log("path: "+JSON.stringify(pointsJSON));
      
        return pointsJSON

      /*
      console.log(route.overview_polyline)
      console.log("path:")
      for(var i = 0; i < route.overview_path.length; i++){
        var point = route.overview_path[i]
        var pointArr = [point.lat(), point.lng()]
        points.push(pointArr)
        pointsJSON.points.push({
          "lat": point.lat(),
          "lng": point.lng()
        }) 
      }
      console.log(JSON.stringify(pointsJSON));
      
      return pointsJSON*/

    }

