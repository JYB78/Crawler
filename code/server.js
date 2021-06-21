// Required module
var dgram = require('dgram');
var http = require('http');
var url = require('url');
var fs = require('fs');
var level = require('level');


/////////////////////////////////// Task 1: Set up UDP socket to talk to ESP32 & print receive message to console

// Port and IP (of RPi) -> For USP Socket
var PORT = 3333;
var HOST = '192.168.1.48';

// Create socket
var server = dgram.createSocket('udp4');
var myport;
var myesp;
var msg;
var key = 0;
var value;

// Create DB
 var db = level('./mydb', {valueEncoding: 'json'});

// Delete old data file
try {
  fs.unlinkSync("./data.csv");
  console.log("Old data removed");
  //file removed
} catch(err) {
  console.error(err)
}

////////////////////////////////////////////////////////////// Task 1: Create UDP Socket (IP)
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    //console.log(remote.address + ':' + remote.port +' - ' + message);
    myport = remote.port;
    myesp = remote.address;
    var line  = message.toString().split(",");
    if(line[0]=="curSplit"){
      value = line[1];
///////////////////////////////////////////////////////////// Task 3: Store current split ti database
      db.put([key], value, function (err) {
        if (err) return console.log('Ooops!', err)
      })
      key++;
      console.log("Inside DB:");
      readDB();       // Read everything inside the DB && print it to console
      fs.appendFile('log.csv', message+'\n', function (err) {
       if (err) throw err;
       //console.log('Updated!');
     });
   } else {
     fs.appendFile('data.csv', message+'\n', function (err) {
      if (err) throw err;
      //console.log('Updated!');
    });
   }
});

// Bind server to port and IP
server.bind(PORT, HOST);


function readDB(arg) {
  db.createReadStream()
  .on('data', function (data) {
      console.log(data.key, '=', data.value)
      // Parsed the data into a structure but don't have to ...
      //var dataIn = {[data.key]: data.value};
      // Stream data to client
      //io.emit('message', dataIn);
    })
    .on('error', function (err) {
      console.log('Oh my!', err)
    })
    .on('close', function () {
      console.log('Stream closed')
    })
    .on('end', function () {
      console.log('Stream ended')
    })
}


/////////////////////////////////// Task 2: Set up server to host html && handle POST requests (send message to ESP32)

http.createServer(function (req, res) {
  var q = url.parse(req.url, true);
  var filename = "." + q.pathname;
  if(filename=="./"){
    fs.readFile("./index.html", function(err, data) {
      if (err) {
        res.writeHead(404, {'Content-Type': 'text/html'});
        return res.end("404 Not Found");
      }
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      return res.end();
    });
  } else if(filename=="./b_front"){
    server.send("F", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT F');
  } else if(filename=="./b_back"){
    server.send("B", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT B');
  } else if(filename=="./b_stop"){
    server.send("Stop", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT Stop');
  } else if(filename=="./b_left"){
    server.send("L", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT L');
  } else if(filename=="./b_right"){
    server.send("R", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT R');
  } else if(filename=="./b_enter-1"){
    server.send("s1", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT s1');
  } else if(filename=="./b_enter-3"){
    server.send("s2", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT s2');
  } else if(filename=="./b_enter-5"){
    server.send("s3", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT s3');
  } else if(filename=="./b_center"){
    server.send("C", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT C');
  } else if(filename=="./b_auto"){
    server.send("A", myport, myesp, function(error){if(error){console.log('ERROR!');}});
    console.log('GOT A');
  } else if(filename=="./data"){
    fs.readFile("./data.csv", function(err, data) {
      if (err) {
        res.writeHead(404, {'Content-Type': 'text/plain'});
        return res.end("404 Not Found");
      }
      res.writeHead(200, {'Content-Type': 'text/plain'});
      res.write(data);
      return res.end();
      });
  }else if(filename=="./log"){
    fs.readFile("./log.csv", function(err, data) {
      if (err) {
        res.writeHead(404, {'Content-Type': 'text/plain'});
        return res.end("404 Not Found");
      }
      res.writeHead(200, {'Content-Type': 'text/plain'});
      res.write(data);
      return res.end();
      });
  }
}).listen(8082);
