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

        for (var i = 0; i < testMarkers.length; i++) {
          testMarkers[i].setMap(null);
        }
        // clear array
        testMarkers = [];
        
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
              var data = getRoutePoints(route);
              logRoute(route, data);
              sendToApp(data);
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
      return JSON.stringify(pointsJSON)
    }

    var testMarkers = [];
    function logRoute(route, data){
      console.log("JSON data: "+data);

      for(var i = 0; i < route.legs.length; i++){
        var legSteps = route.legs[i].steps
        for (var k = 0; k < legSteps.length; k++){
          var step = legSteps[k].path
          for(var j = 0; j < step.length; j++){
            var point = step[j];
            var newMarker = new google.maps.Marker({
              position: {lat: point.lat(), lng: point.lng()},
              map: map,
              draggable: false,
              label: { 
                text: j.toString(),
                color: 'white',
                fontSize: "10px"

              },
              icon: {
                path: google.maps.SymbolPath.CIRCLE,
                fillColor: (k%2==0)? 'red' : 'blue',
                fillOpacity: .4,
                scale: 5,
                strokeColor: 'white',
                strokeWeight: 1  
              }
          });
            testMarkers.push(newMarker);

          }
        }
      }
    }


 function sendToApp(postData){
      console.log(postData);
      //var url = "http://localhost:8000/secretdatatransfer";
      var url = "https://" + window.location.hostname + "/secretdatatransfer";
      var method = "POST";

// You REALLY want async = true.
// Otherwise, it'll block ALL execution waiting for server response.
var async = true;

var request = new XMLHttpRequest();

// Before we send anything, we first have to say what we will do when the
// server responds. This seems backwards (say how we'll respond before we send
// the request? huh?), but that's how Javascript works.
// This function attached to the XMLHttpRequest "onload" property specifies how
// the HTTP response will be handled. 
request.onload = function () {

   // Because of javascript's fabulous closure concept, the XMLHttpRequest "request"
   // object declared above is available in this function even though this function
   // executes long after the request is sent and long after this function is
   // instantiated. This fact is CRUCIAL to the workings of XHR in ordinary
   // applications.

   // You can get all kinds of information about the HTTP response.
   var status = request.status; // HTTP response status, e.g., 200 for "200 OK"
   var data = request.responseText; // Returned data, e.g., an HTML document.
 }

 request.open(method, url, async);

 request.setRequestHeader("Content-Type", "application/json;charset=UTF-8");

// Actually sends the request to the server.
request.send(postData);
}

