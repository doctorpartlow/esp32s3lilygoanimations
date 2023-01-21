
//if you want to run this on the lilygo esp32 s3 t-display you'll probably have to clone
//the whole respository just open this folder with vs code using platformio and upload it. it should 
//work with the directory structure in place from the repo.

//there's a couple sets of global variables in the code. if you change anything just be aware of that as I'd rather write a caution message then put them in a proper spot. 
//I'm lazy and this this isn't really anything that's business critical so I'm not going to worry about it. but if you have some type of ailment that makes you fix things then go ahead and fix it. 
//I'm sure the time it took to write this I could have fixed the code but then there wouldn't be so many comments everywhere to read. 

#include <Arduino.h>
#include <TFT_eSPI.h>/* Please use the TFT library provided in the library. */
#include <pin_config.h>
#include <ctime>
TFT_eSPI tft = TFT_eSPI();
#define WAIT 1000
unsigned long targetTime = 0; // Used for testing draw times
static const uint16_t NUM_PARTICLES = 175;
uint16_t ROWS = 320;
uint16_t COLS = 170;
boolean loadingflag = true;
bool firstPass = true;
float separa = 2.2F;
float alignm = 1.3F;
float cohesi = 1.2F;
int desiredseparation = 3;
int neighbordist = 6;

template <class T>
class vec2 {
public:
	T x, y;
	
	vec2() :x(0), y(0) {}
	vec2(T x, T y) : x(x), y(y) {}
	vec2(const vec2& v) : x(v.x), y(v.y) {}
	
	vec2& operator=(const vec2& v) {
		x = v.x;
		y = v.y;
		return *this;
	}
    bool operator==(vec2& v) {
        return x == v.x && y == v.y;
    }

    bool operator!=(vec2& v) {
        return !(x == y);
    }
	
	vec2 operator+(vec2& v) {
		return vec2(x + v.x, y + v.y);
	}
	vec2 operator-(vec2& v) {
		return vec2(x - v.x, y - v.y);
	}
	
	vec2& operator+=(vec2& v) {
		x += v.x;
		y += v.y;
		return *this;
	}
	vec2& operator-=(vec2& v) {
		x -= v.x;
		y -= v.y;
		return *this;
	}	
	vec2 operator+(double s) {
		return vec2(x + s, y + s);
	}
	vec2 operator-(double s) {
		return vec2(x - s, y - s);
	}
	vec2 operator*(double s) {
		return vec2(x * s, y * s);
	}
	vec2 operator/(double s) {
		return vec2(x / s, y / s);
	}
	
	
	vec2& operator+=(double s) {
		x += s;
		y += s;
		return *this;
	}
	vec2& operator-=(double s) {
		x -= s;
		y -= s;
		return *this;
	}
	vec2& operator*=(double s) {
		x *= s;
		y *= s;
		return *this;
	}
	vec2& operator/=(double s) {
		x /= s;
		y /= s;
		return *this;
	}
	
	void set(T x, T y) {
		this->x = x;
		this->y = y;
	}
	
	void rotate(double deg) {
		double theta = deg / 180.0 * M_PI;
		double c = cos(theta);
		double s = sin(theta);
		double tx = x * c - y * s;
		double ty = x * s + y * c;
		x = tx;
		y = ty;
	}
	
	vec2& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}
	
	float dist(vec2 v) const {
		vec2 d(v.x - x, v.y - y);
		return d.length();
	}
	float length() const {
		return sqrt(x * x + y * y);
	}
	void truncate(double length) {
		double angle = atan2f(y, x);
		x = length * cos(angle);
		y = length * sin(angle);
	}
	
	vec2 ortho() const {
		return vec2(y, -x);
	}

  void rotateAround(const vec2 &rhs, float radiant)
  {
      vec2 cpy = rhs;
      vec2 diff = this - rhs;
      float _cos = cos(radiant);
      float _sin = sin(radiant);
      x = cpy.x + diff.x * _cos - diff.y * _sin;
      y = cpy.x + diff.x * _sin + diff.y * _cos;
  }

  vec2 linearInterpolation(const vec2 &rhs, float percent) // percent must be a value between 0 and 1
  {
    if (percent < 0 || percent > 1)
        return vec2(0,0);
     return this + ((rhs - this) * percent);
  }
  float angle(const vec2 &rhs) const
  {
      float tmp = atan2(rhs.y - y, rhs.x - x);
      tmp = tmp * 180 / PI;
      if (tmp < 0)
          tmp += 360;
      return tmp;
  };
	static float dot(vec2 v1, vec2 v2) {
		return v1.x * v2.x + v1.y * v2.y;
	}
	static float cross(vec2 v1, vec2 v2) {
		return (v1.x * v2.y) - (v1.y * v2.x);
	}
  const vec2 perpCCW() const
	{
		return vec2(-y, x);
	}

	const vec2 perpCW() const
	{
		return vec2(y, -x);
	}

  float mag() const {
      return length();
  }

  float magSq() {
        return (x * x + y * y);
  }

  void limit(float max) {
    if (magSq() > max*max) {
        normalize();
        *this *= max;
    }
  }	
};

typedef vec2<float> PVector;
typedef vec2<double> vec2d;

//here's another set of globals in the middle of the code. if you do this you're lazy like me.
double maxforce;    // Maximum steering force
double maxspeed;    // Maximum speed

class Boid {
  public:
    PVector location;
    PVector velocity;
    PVector acceleration;
    int hue;
    byte colorIndex = 0;
    float mass;
    uint8_t size = 0;
    int startingsize = 0;
    boolean enabled = true;
    uint8_t id = 1;
    PVector lastLocation;
    Boid() {}

    Boid(float x, float y) {
      
      acceleration = PVector(0, 0);
      
      velocity = PVector(randomf(), randomf());
      location = PVector(x, y);
      location.rotate(3);
      //lastlocation = location;
      maxspeed = random(1.1,2.2);
      maxforce = random(1.5,2.5);
      startingsize = random(3,10);
      size = random(3,10);
      mass = random(1.0,3.5);
      hue = random(10000,65536);
      id += 1;
    }

    static float randomf() {
      return mapfloat(random(0, 255), 0, 255, -.5, .5);
    }

    static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void run(Boid boids [], uint8_t boidCount) {
      flock(boids, boidCount);
      //location.rotate(3);
    }
    // Method to update location
    void update(Boid boids [], uint8_t boidCount) {
      lastLocation = location;
      flock(boids, boidCount);
      velocity += acceleration;
      velocity.limit(maxspeed);
      location += velocity;      
      acceleration *= 0;
     // location.rotate(0.5);
      //location.ortho();
    }


    void applyForce(PVector force) {
      // We could add mass here if we want A = F / M
      acceleration += force;      
    }

    void repelForce(PVector obstacle, float radius) {
      //Force that drives boid away from obstacle.
      PVector futPos = location + velocity; //Calculate future position for more effective behavior.
      PVector dist = obstacle - futPos;
      float d = dist.mag();
      if (d <= radius) {
        PVector repelVec = location - obstacle;
        repelVec.normalize();
        if (d != 0) { //Don't divide by zero.
          repelVec.normalize();
          repelVec *= (maxforce * 7);
          if (repelVec.mag() < 0) { //Don't let the boids turn around to avoid the obstacle.
            repelVec.y = 0;
          }
        }
        applyForce(repelVec);
      }
    }
    // We accumulate a new acceleration each time based on three rules
    void flock(Boid boids [], uint8_t boidCount) {
      PVector sep = separate(boids, boidCount);   // Separation
      PVector ali = align(boids, boidCount);      // Alignment
      PVector coh = cohesion(boids, boidCount);   // Cohesion
      // Arbitrarily weight these forces
      sep *= separa; //2.8; //change these to change the behavior of the flock 
      ali *= alignm; //1.2; // 
      coh *= cohesi;//1.2; //
      // Add the force vectors to acceleration
      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
    }

    // Separation
    // Method checks for nearby boids and steers away
    PVector separate(Boid boids [], uint8_t boidCount) {
      PVector steer = PVector(0, 0);
      int count = 0;
      // For every boid in the system, check if it's too close
      for (int i = 0; i < boidCount; i++) {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((d > 0) && (d < desiredseparation)) {
          // Calculate vector pointing away from neighbor
          PVector diff = location - other.location;
          diff.normalize();
          diff /= d;        // Weight by distance
          steer += diff;
          count++;            // Keep track of how many
        }
      }
      // Average -- divide by how many
      if (count > 0) {
        steer /= (float) count;
      }
      // As long as the vector is greater than 0
      if (steer.mag() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer *= maxspeed;
        steer -= velocity;
        steer.limit(maxforce);
      }
      return steer;
    }
    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    PVector align(Boid boids [], uint8_t boidCount) {
      PVector sum = PVector(0, 0);
      int count = 0;
      for (int i = 0; i < boidCount; i++) {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        if ((d > 0) && (d < neighbordist)) {
          sum += other.velocity;
          count++;
        }
      }
      if (count > 0) {
        sum /= (float) count;
        sum.normalize();
        sum *= maxspeed;
        PVector steer = sum - velocity;
        steer.limit(maxforce);
        return steer;
      }
      else {
        return PVector(0, 0);
      }
    }
    // Cohesion
    // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
    PVector cohesion(Boid boids [], uint8_t boidCount) {
      PVector sum = PVector(0, 0);   // Start with empty vector to accumulate all locations
      int count = 0;
      for (int i = 0; i < boidCount; i++) {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        if ((d > 0) && (d < neighbordist)) {
          sum += other.location; // Add location
          count++;
        }
      }
      if (count > 0) {
        sum /= count;
        return seek(sum);  // Steer towards the location
      }
      else {
        return PVector(0, 0);
      }
    }
    // A method that calculates and applies a steering force towards a target    
    PVector seek(PVector target) {
      PVector desired = target - location;  // A vector pointing from the location to the target
      // Normalize desired and scale to maximum speed
      desired.normalize();
      desired *= maxspeed;
      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce);  // Limit to maximum steering force
      return steer;
    }
    // A method that calculates a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    void arrive(PVector target) {
      PVector desired = target - location;  // A vector pointing from the location to the target
      float d = desired.mag();
      // Normalize desired and scale with arbitrary damping within 100 pixels
      desired.normalize();
      if (d < 4) {
        float m = map(d, 0, 100, 0, maxspeed);
        desired *= m;
      }
      else {
        desired *= maxspeed;
      }
      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce);  // Limit to maximum steering force
      applyForce(steer);
      //Serial.println(d);
    }
        void wrapAroundBorders() {
      if (location.x < 0) location.x = ROWS - 1;
      if (location.y < 0) location.y = COLS - 1;
      if (location.x >= ROWS) location.x = 0;
      if (location.y >= COLS) location.y = 0;
    }
    void avoidBorders() {
      PVector desired = velocity;

      if (location.x < 8) desired = PVector(maxspeed, velocity.y);
      if (location.x >= ROWS - 8) desired = PVector(-maxspeed, velocity.y);
      if (location.y < 8) desired = PVector(velocity.x, maxspeed);
      if (location.y >= COLS - 8) desired = PVector(velocity.x, -maxspeed);

      if (desired != velocity) {
        PVector steer = desired - velocity;
        steer.limit(maxforce);
        applyForce(steer);
      }

      if (location.x < 0) location.x = 0;
      if (location.y < 0) location.y = 0;
      if (location.x >= ROWS) location.x = ROWS - 1;
      if (location.y >= COLS) location.y = COLS - 1;
    }
};
//that's right there's globals right in the middle of the code.
const int LONG_PRESS_TIME1  = 1000; // 1000 milliseconds
int lastState1 = LOW;  // the previous state from the input pin
int currentState1;     // the current reading from the input pin
unsigned long pressedTime1  = 0;
bool isPressing1 = false;
bool isLongDetected1 = false;
const int LONG_PRESS_TIME2  = 1000; // 1000 milliseconds
int lastState2 = LOW;  // the previous state from the input pin
int currentState2;     // the current reading from the input pin
unsigned long pressedTime2  = 0;
bool isPressing2 = false;
bool isLongDetected2 = false;

void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);  
  pinMode(PIN_BUTTON_1,INPUT);
  pinMode(PIN_BUTTON_2,INPUT);
  
  tft.begin(); 
  tft.fillScreen(TFT_BLACK); 
  delay(2000);
  tft.setTextSize(1);
}
float lastx[NUM_PARTICLES];
float lasty[NUM_PARTICLES];
float dify[NUM_PARTICLES];
float difx[NUM_PARTICLES];
int boidspeed = 0;
float lastboidsize[NUM_PARTICLES];
uint32_t colors = 0x0;
bool loadingFlag = true;
Boid boids[NUM_PARTICLES];       
int speed = 1; 
int count = NUM_PARTICLES;

void start() {
  for (int i = 0; i < NUM_PARTICLES; i++) {
  boids[i] = Boid(random(ROWS), random(COLS));
  separa = (float)random(1.11F, 3.99F);
  alignm = (float)random(1.11F,3.99F);
  cohesi = (float)random(1.11F,3.99F);
  maxspeed = random(1.1,15.5);
  maxforce = random(1.5,15.5);
  desiredseparation = random(2,12);
  neighbordist = random(5,20);

  }
}



void newRandom(){
    separa = (float)random(1.11F, 5.99F);
    alignm = (float)random(1.11F,5.99F);
    cohesi = (float)random(1.11F,5.99F);
    maxspeed = random(1.1,10.5);
    maxforce = random(1.5,10.5);
    desiredseparation = random(2,20);
    neighbordist = random(3,20);
}

void draw(){

  if (firstPass)
  {
    firstPass = false;
    start();     
   }
  
  for (int i = 0; i < NUM_PARTICLES; i++) {
       
    Boid * boid = &boids[i];
     tft.drawLine(lasty[i],lastx[i],boid->lastLocation.y,boid->lastLocation.x,0X000000);
   // tft.drawPixel(lasty[i],lastx[i],0x000000);
    boid->update(boids, NUM_PARTICLES);
    //boid->wrapAroundBorders(); 
    boid->avoidBorders();
        lastx[i] = boid->location.x;
    lasty[i] = boid->location.y;      
        tft.drawPixel(boid->location.y,boid->location.x,boid->hue);  
        tft.drawLine(boid->location.y,boid->location.x,boid->lastLocation.y,boid->lastLocation.x,boid->hue);
  //    tft.drawLine(boid->location.y,boid->location.x,(int32_t)lastx,(int32_t)lasty,boid->hue);
    difx[i] = boid->location.x - lastx[i];
    dify[i] = boid->location.y - lasty[i];      
    lastboidsize[i] = boid->size;   
  }
}
unsigned long last; // last time we fired
unsigned long interval = 5000; // "delay" till next firing
unsigned long diff; 
void loop(){
unsigned long now = millis(); 
diff = now - last;
if (diff > interval){
  last = now;
  newRandom();
}
draw();

//this long press donesn't work if you feel like fixing it by all means but the short button press
//changes all of the varaibles each time either of the two are pressed.
 currentState1 = digitalRead(PIN_BUTTON_1);

  if(lastState1 == HIGH && currentState1 == LOW) {        // button is pressed
    pressedTime1 = millis();
    isPressing1 = true;
    isLongDetected1 = false;
  } else if(lastState1 == LOW && currentState1 == HIGH) { // button is released
    isPressing1 = false;
  }

  if(isPressing1 == true && isLongDetected1 == false) {
    long pressDuration = millis() - pressedTime1;

    if( pressDuration > LONG_PRESS_TIME1 ) {
      newRandom();
      isLongDetected1 = true;
    }
    else{
      newRandom();
    }
  }
 currentState2 = digitalRead(PIN_BUTTON_2);

  if(lastState2 == HIGH && currentState2 == LOW) {        // button is pressed
    pressedTime2 = millis();
    isPressing2 = true;
    isLongDetected2 = false;
  } else if(lastState2 == LOW && currentState2 == HIGH) { // button is released
    isPressing2 = false;
  }
  if(isPressing2 == true && isLongDetected2 == false) {
    long pressDuration = millis() - pressedTime2;

    if( pressDuration > LONG_PRESS_TIME2 ) {
      newRandom();
      isLongDetected2 = true;      
    }else
    {
      newRandom();      
    }
  }
lastState1 = currentState1;
lastState2 = currentState2;
  tft.setCursor(0,0);
tft.print("s:");
tft.println(separa);
tft.print("c:");
tft.println(cohesi);
tft.print("a:");
tft.println(alignm);
tft.print("d:");
tft.println(desiredseparation);    
tft.print("d:");
tft.println(neighbordist);
tft.print("m:");
tft.println(maxspeed);
tft.print("f:");
tft.println(maxforce);       

    
} 
