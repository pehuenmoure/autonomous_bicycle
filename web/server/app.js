var express = require('express');
var app = express();
var http = require('http');
var server = http.Server(app);
var io = require('socket.io')(server);
var SerialPort = require('serialport');

//For Writing Data
var fileWriter = require('msexcel-builder');
var testNo = 0;
var name = new String("data");
var fileName = name.concat(String(testNo)).concat('.csv');
var row = 1;
var col = 1;
console.log(fileName);

//For getting input- Will be edited out once site is nicer
var prompt = require('prompt');
prompt.start();



//======================rest of code=======================
app.set('view engine', 'ejs');

var SerialPort = require('serialport');
SerialPort.list(function (err, ports) {

	ports.forEach(function(port) {

		console.log(port.comName);
		console.log(port.pnpId);
		console.log(port.manufacturer);
		if(port.manufacturer.includes('Arduino')){
			var myPort = new SerialPort(port.comName, {
				parser: SerialPort.parsers.readline('\n')
			});

			io.on('connect', function(socket){
				console.log('a user connected');
			});

			var data = Array(6);
			var i = 0;
			myPort.on('data', function(d){
				if (i == 4) {
					console.log(data);
					io.emit('data-msg', data);
					i = 0;
				} else{
					data[i] = parseFloat(d);
					i ++;
				}
			});
			setPrompt();
			//=========================FOR INPUTTING DATA TO ARDUINO====================
			function setPrompt(){
				prompt.get(['input'], function (err, result) {
					if (err) { return onErr(err); }
					console.log('Command-line input received:');
					console.log(result.input);
					if(result.input==='start'){
						myPort.write('y');
						setPrompt();
					}else if(result.input==='stop'){
						myPort.write('n');
						setPrompt();
					}
				});
			}

			function onErr(err) {
				console.log(err);
				return 1;
			}
			

		}
	});
});


//========================FOR WRITING .CSV FILES=========================
// Create a new workbook file in current working-path 
var workbook = fileWriter.createWorkbook('./', fileName);

  // Create a new worksheet with 10 columns and 12 rows 
  var sheet1 = workbook.createSheet('sheet1', 10, 12);
  
  // Fill some data 
  sheet1.set(1, 1, 'I am title');
  for (var i = 2; i < 5; i++)
  	sheet1.set(i, 1, 'test'+i);
  
  // Save it 
  workbook.save(function(ok){
  	if (!ok) 
  		workbook.cancel();
  	else
  		console.log('congratulations, your workbook created');
  });




  //========================================================================

  
  function writeAndDrain (data, callback) {
  	sp.write(data, function () {
  		sp.drain(callback);
  	});
  }


  app.get('/', function(req, res){
  	res.render('index');
  });

  server.listen(8000, function(){
  	console.log('listening on 8000');
  });
