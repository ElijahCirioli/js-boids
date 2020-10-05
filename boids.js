//setup canvas
var canvas = document.getElementById("myCanvas");
var context = canvas.getContext("2d");

//initialize variables
var boids = []; //all the boids
var distance = 100; //the radius of the boid's neighborhood (px)
var viewAngle = 5 * Math.PI / 3; //the viewing angle of the boid's neighborhood (radians)
var boidCount = 70; //how many boids to create
var speed = 4.0; //how fast the boids move
var showDebug = false; //show view angle and rays for one boid
var mouseMode = 0; //0 = draw boids, 1 = scatter
var mouseDown = false; //is the mouse being pressed down
var pred = new Predator(0, 0, 0); //the predator to scatter the boids away from

//define weights for the three rules
var separationWeight = 1;
var alignmentWeight = 1;
var cohesionWeight = 1;
//how much the rules affect the current velocity
var inertia = 0.5;

//define a boid
function Boid(x, y, velX, velY) {
  this.x = x;
  this.y = y;
  this.direction = 0;
  this.velX = velX;
  this.velY = velY;
  this.neighborhood = [];
}

//define a predator
function Predator(x, y, life) {
  this.x = x;
  this.y = y;
  this.life = life;
}

//game cycle
function update() {
  moveBoids();
  draw();
}

//update the velocity vectors of all boids and move
function moveBoids() {
  for (var i = 0; i < boids.length; i++) {
    var b = boids[i];
    b.neighborhood = getNeighborhood(b); //get all boids within Distance radius and viewAngle

    //gather the vectors from the three rules
    var vec1 = b.separation();
    var vec2 = b.alignment();
    var vec3 = b.cohesion();
    var vec4 = b.guidance();

    //add the vectors together and multiply weights
    b.velX += inertia * ((vec1[0] * separationWeight) + (vec2[0] * alignmentWeight) + (vec3[0] * cohesionWeight) + vec4[0]);
    b.velY += inertia * ((vec1[1] * separationWeight) + (vec2[1] * alignmentWeight) + (vec3[1] * cohesionWeight) + vec4[1]);

    //normalize the speed to what it should be
    var nomalizedVector = normalize([b.velX, b.velY], speed);
    b.velX = nomalizedVector[0];
    b.velY = nomalizedVector[1];

    //move boid
    b.x += b.velX;
    b.y += b.velY;

    //set direction
    b.direction = Math.atan2(b.velY, b.velX);
    if (b.direction < 0) {
      b.direction += (2 * Math.PI);
    }

    var buffer = 20; //how far outside of the screen before they loop back
    //loop X coords around
    if (b.x > canvas.width + buffer) {
      b.x = -buffer;
    } else if (b.x < -buffer) {
      b.x = canvas.width + buffer;
    }
    //loop Y coords around
    if (b.y > canvas.height + buffer) {
      b.y = -buffer;
    } else if (b.y < -buffer) {
      b.y = canvas.height + buffer;
    }
  }
}

//render the scene
function draw() {
  //draw background
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = "#27303d";
  context.fillRect(0, 0, canvas.width, canvas.height);

  //draw all boids
  for (var i = 0; i < boids.length; i++) {
    var b = boids[i];
    context.save();
    context.translate(b.x, b.y);
    context.rotate(b.direction);
    //draw single boid
    drawBoid("#628754");
    context.restore();
  }

  //draw predator
  if (pred.life > 0) {
    context.strokeStyle = "rgba(255, 204, 0, " + pred.life / 20 + ")";
    context.lineWidth = 3;
    context.beginPath();
    context.arc(pred.x, pred.y, 30, 0, 2 * Math.PI);
    context.stroke();
    pred.life--;
  }

  //draw debug
  if (showDebug) {
    context.save();
    context.translate(boids[0].x, boids[0].y);
    context.rotate(boids[0].direction);
    context.fillStyle = "rgba(255, 255, 255, 0.25)";
    context.beginPath()
    context.arc(0, 0, distance, -viewAngle / 2, viewAngle / 2);
    context.lineTo(0, 0);
    context.fill();
    drawBoid("red");
    context.restore();

    context.strokeStyle = "red";
    context.lineWidth = 1;
    for (var j = 0; j < boids[0].neighborhood.length; j++) {
      context.beginPath();
      context.moveTo(boids[0].x, boids[0].y);
      context.lineTo(boids[0].neighborhood[j].x, boids[0].neighborhood[j].y);
      context.stroke();
    }
  }
}

//draw little boid triangle
function drawBoid(color) {
  context.fillStyle = color;
  context.beginPath();
  context.moveTo(10, 0);
  context.lineTo(-10, 6);
  context.lineTo(-8, 0);
  context.lineTo(-10, -6);
  context.lineTo(10, 0);
  context.fill();
}

//return distance between two points
function getDistance(x1, y1, x2, y2) {
  return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
}

//get all neighboring boids for a given boid
function getNeighborhood(b) {
  var neighborhood = [];
  for (var i = 0; i < boids.length; i++) { //loop through all boids
    if (b != boids[i]) { //if it isn't the current boid
      if (getDistance(b.x, b.y, boids[i].x, boids[i].y) <= distance) { //if the boid is within the radius
        var angle = Math.atan2((b.y - boids[i].y), (b.x - boids[i].x)); //calc angle between two boids
        if (angle < 0) { //convert from (-pi, pi) to (0, 2pi)
          angle += (Math.PI * 2)
        }
        var relativeAngle = Math.abs(b.direction - angle); //see how close the two angles are
        if (relativeAngle > Math.PI - (viewAngle / 2)) {
          neighborhood.push(boids[i]); //add to boid's neighborhood
        }
      }
    }
  }
  return neighborhood;
}

//steer to avoid crowding neighbors
Boid.prototype.separation = function() {
  var vec = [0, 0];
  for (var i = 0; i < this.neighborhood.length; i++) {
    var localDist = getDistance(this.x, this.y, this.neighborhood[i].x, this.neighborhood[i].y);
    if (localDist > 0) {
      if (Math.abs(this.neighborhood[i].x - this.x) > 0) {
        vec[0] -= ((this.neighborhood[i].x - this.x) / localDist) * (distance / localDist);
      }
      if (Math.abs(this.neighborhood[i].y - this.y) > 0) {
        vec[1] -= ((this.neighborhood[i].y - this.y) / localDist) * (distance / localDist);
      }
    }
  }
  vec = normalize(vec, 1);
  return vec;
}

//steer to match alignment with neighbors
Boid.prototype.alignment = function() {
  var vec = [0, 0];
  for (var i = 0; i < this.neighborhood.length; i++) {
    vec[0] += this.neighborhood[i].velX;
    vec[1] += this.neighborhood[i].velY;
  }
  vec = normalize(vec, 1);
  return vec;
}

//steer towards the center of mass of neighbors
Boid.prototype.cohesion = function() {
  var vec = [0, 0];
  var centerX = 0,
    centerY = 0;
  if (this.neighborhood.length > 0) {
    for (var i = 0; i < this.neighborhood.length; i++) {
      centerX += this.neighborhood[i].x;
      centerY += this.neighborhood[i].y;
    }
    centerX /= this.neighborhood.length;
    centerY /= this.neighborhood.length;

    var vec = [centerX - this.x, centerY - this.y]
    vec = normalize(vec, 1);
  }
  return vec;
}

//steer towards the mouse
Boid.prototype.guidance = function() {
  var vec = [0, 0];
  if (pred.life > 0) {
    vec[0] = this.x - pred.x;
    vec[1] = this.y - pred.y;
  }
  vec = normalize(vec, 1);
  return vec;
}

//change the length of a vector to the scale
function normalize(vec, scale) {
  var len = getDistance(0, 0, vec[0], vec[1]);
  if (len > 0) {
    return [scale * vec[0] / len, scale * vec[1] / len];
  } else {
    return [0, 0];
  }
}

//initialize all boids
function init() {
  for (var i = 0; i < boidCount; i++) {
    var x = 50 + (Math.random() * 900);
    var y = 50 + (Math.random() * 600);
    var velX = (Math.random() * 6) - 3;
    var velY = (Math.random() * 6) - 3;

    var b = new Boid(x, y, velX, velY);
    boids.push(b);
  }
}

//mouse controls
canvas.onclick = function(e) {
  var mX = e.offsetX;
  var mY = e.offsetY;
  if (mouseMode === 1) {
    pred.x = mX;
    pred.y = mY;
    pred.life = 30;
  }
}
canvas.onmousedown = function(e) {
  mouseDown = true;
  if (mouseMode === 0) {
    var mX = e.offsetX;
    var mY = e.offsetY;
    var velX = (Math.random() * 6) - 3;
    var velY = (Math.random() * 6) - 3;

    var b = new Boid(mX, mY, velX, velY);
    boids.push(b);
  }
}
canvas.onmouseup = function(e) {
  mouseDown = false;
}
var mouseCooldown = 0;
canvas.onmousemove = function(e) {
  if (mouseDown && mouseMode === 0) {
    var mX = e.offsetX;
    var mY = e.offsetY;

    if (mouseCooldown === 0) {
      var velX = (Math.random() * 6) - 3;
      var velY = (Math.random() * 6) - 3;

      var b = new Boid(mX, mY, velX, velY);
      boids.push(b);
      mouseCooldown = 5;
			cd();
    }
  }
}
function cd() {
	mouseCooldown--;
	if (mouseCooldown > 0) {
		setTimeout(cd, 10);
	}
}

//reset button
document.getElementById("resetButton").onclick = function(e) {
  separationWeight = 1;
  alignmentWeight = 1;
  cohesionWeight = 1;
  inertia = 0.5;
  speed = 4.0;
  document.getElementById("speedSlider").value = 40;
  document.getElementById("inertiaSlider").value = 50;
	triX = 80;
	triY = 58.33;
  boids = [];
	drawTriangle();
  init();
}
//mouse mode buttons
document.getElementById("createButton").onclick = function(e) {
  mouseMode = 0;
  document.getElementById("scatterButton").style.backgroundColor = "#c9c9c9";
  document.getElementById("createButton").style.backgroundColor = "#506E45";
  document.getElementById("scatterButton").style.color = "black";
  document.getElementById("createButton").style.color = "white";
}
document.getElementById("scatterButton").onclick = function(e) {
  mouseMode = 1;
  document.getElementById("createButton").style.backgroundColor = "#c9c9c9";
  document.getElementById("scatterButton").style.backgroundColor = "#506E45";
  document.getElementById("scatterButton").style.color = "white";
  document.getElementById("createButton").style.color = "black";
}
//debug checkbox
document.getElementById("debugButton").onclick = function(e) {
  showDebug = !showDebug;
  if (showDebug) {
    document.getElementById("debugButton").style.backgroundColor = "#506E45";
  } else {
    document.getElementById("debugButton").style.backgroundColor = "#c9c9c9";
  }
}
//sliders
document.getElementById("speedSlider").oninput = function(e) {
  speed = document.getElementById("speedSlider").value / 10;
}
document.getElementById("inertiaSlider").oninput = function(e) {
  inertia = 1 - (document.getElementById("inertiaSlider").value / 100);
}

//triangle selector
var triCan = document.getElementById("triCanvas");
var triCon = triCanvas.getContext("2d");
var triX = 80;
var triY = 58.33;
var triMouseDown = false;

function drawTriangle() {
	triCon.clearRect(0, 0, triCan.width, triCan.height);
	triCon.strokeStyle = "#506E45";
	triCon.fillStyle = "#c9c9c9";
	triCon.lineWidth = 3;
	triCon.beginPath();
	triCon.moveTo(22.265, 25);
	triCon.lineTo(80, 125);
	triCon.lineTo(137.735, 25);
	triCon.lineTo(22.265, 25);
	triCon.fill();
	triCon.stroke();
	
	triCon.fillStyle = "#506E45";
	triCon.lineWidth = 1;
	triCon.beginPath();
	triCon.arc(80, 58.33, 2, 0, 2 * Math.PI);
	triCon.fill();
	
	triCon.fillStyle = "#27303d";
	triCon.beginPath();
	triCon.arc(triX, triY, 7, 0, 2 * Math.PI);
	triCon.fill();
	
	triCon.fillStyle = "#383838";
	triCon.textAlign = "center";
	triCon.font = "11px Arial";
	triCon.fillText("Sep", 22.265, 8);
	triCon.fillText(separationWeight.toFixed(2), 22.265, 19);
	triCon.fillText("Ali", 137.735, 8);
	triCon.fillText(alignmentWeight.toFixed(2), 137.735, 19);
	triCon.fillText("Coh", 80, 138);
	triCon.fillText(cohesionWeight.toFixed(2), 80, 149);
}
triCan.onmousedown = function(e) {
	var mX = e.offsetX;
	var mY = e.offsetY;
	triMouseDown = true;
	moveTri(mX, mY);
	drawTriangle();
}
triCan.onmouseup = function(e) {
	triMouseDown = false;
}
triCan.onmouseleave = function(e) {
	triMouseDown = false;
}
triCan.onmousemove = function(e) {
	var mX = e.offsetX;
	var mY = e.offsetY;
	if (triMouseDown) {
		moveTri(mX, mY);
	}
	drawTriangle();
}
function moveTri(x, y) {
	//see if it can move to the new coordinates
	if (x > 22.265 && x < 137.735) {
		if ((x - 22.265) > ((y - 25) / 1.73205161514) && (x - 137.735) < ((y - 25) / -1.73205161514)) {
			triX = x;
		}
	}
	if (y > 25 && y < 125) {
		if ((y - 25) < ((x - 22.265) * 1.73205161514) && (y - 25) < ((x - 137.735) * -1.73205161514)) {
			triY = y;
		}
	}
	
	//adjust the weights
	separationWeight = 2 - (getDistance(triX, triY, 22.265, 25) / 67.37);
	alignmentWeight = 2 - (getDistance(triX, triY, 137.735, 25) / 67.37);
	cohesionWeight = 2 - (getDistance(triX, triY, 80, 125) / 67.37);
}
drawTriangle();

init();
var thread = setInterval(update, 1000 / 60)