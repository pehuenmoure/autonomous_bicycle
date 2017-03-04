var express = require('express');
var bodyParser = require('body-parser');
var app = express();
var path = require('path');
var http = require('http');
var server = http.Server(app);
var SerialPort = require('serialport');
var XMLHttpRequest = require("xmlhttprequest").XMLHttpRequest;


app.use(bodyParser.json()); // for parsing application/json
app.use(express.static(path.join(__dirname, 'public')));


//======================rest of code=======================

var SerialPort = require('serialport');
SerialPort.list(function (err, ports) {

	ports.forEach(function(port) {

		console.log(port.comName);
		console.log(port.pnpId);
		console.log(port.manufacturer);

		if(port.manufacturer.indexOf('arduino')>-1){
			console.log(port.comName);	
			var myPort = new SerialPort(port.comName, {
				parser: SerialPort.parsers.readline('\n')
			});
			//Setting up array that stores incoming data
			var data = new Array(6);
			myPort.on('data', function(d){
				 console.log(d);
				if(d>5000){
					data[5] = d;
					sendToApp(JSON.stringify(data)); //Should I reinitialize the array?
					//console.log(data);
				}
				else{
					var col = Math.floor(Math.trunc(d)/1000);
					data[col-1] = d-(col*1000);
				}

			});

		}
	});
});

server.listen(8080, function(){
	console.log('listening on 8000');
});

function sendToApp(postData){
	console.log(postData);
	var url = "http://localhost:8000/datastream";
	var method = "POST";
	var async = true;

	var request = new XMLHttpRequest();
	request.onload = function () {
  	 var status = request.status; // HTTP response status, e.g., 200 for "200 OK"
   	var data = request.responseText; // Returned data, e.g., an HTML document.
   }

   request.open(method, url, async);

   request.setRequestHeader("Content-Type", "application/json;charset=UTF-8");

// Actually sends the request to the server.
request.send(postData);
}
