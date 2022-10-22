

#include <Arduino.h>
#include <TFT_eSPI.h>/* Please use the TFT library provided in the library. */
#include <img_logo.h>
#include <pin_config.h>

TFT_eSPI tft = TFT_eSPI();
#define WAIT 1000
unsigned long targetTime = 0; // Used for testing draw times
static const uint16_t NUM_PARTICLES = 250;
uint16_t ROWS = 170;
uint16_t COLS = 320;
boolean loadingflag = true;

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
	
	static float dot(vec2 v1, vec2 v2) {
		return v1.x * v2.x + v1.y * v2.y;
	}
	static float cross(vec2 v1, vec2 v2) {
		return (v1.x * v2.y) - (v1.y * v2.x);
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
//<-- end of the vector library
class Boid {
  public:
    PVector location;
    PVector velocity;
    PVector acceleration;
    float maxforce;    // Maximum steering force
    float maxspeed;    // Maximum speed
    int hue;
    float desiredseparation = 8;
    float neighbordist = 14;
    byte colorIndex = 0;
    float mass;
    uint8_t size = 0;
    int startingsize = 0;
    boolean enabled = true;
    uint8_t id = 1;

    Boid() {}

    Boid(float x, float y) {
      acceleration = PVector(0, 0);
      velocity = PVector(randomf(), randomf());
      location = PVector(x, y);
      maxspeed = 5.5;
      maxforce = 1.5;
      startingsize = random(3,10);
      size = random(3,10);
      mass = random(1.0,3.5);
      hue = random(0x0A0A0A0A,0xFFFFFFFF);
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
      update();
    }

    // Method to update location
    void update() {
      // Update velocity
      velocity += acceleration;
      // Limit speed
      velocity.limit(maxspeed);
      location += velocity;
      // Reset acceleration to 0 each cycle
      acceleration *= 0;
      //avoidBorders();
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
      sep *= 1.5;
      ali *= 1.0;
      coh *= 1.5;
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
};
 
 Boid boids[NUM_PARTICLES];    //this makes the boids
 uint16_t x;
 uint16_t y;
 uint16_t z;

 uint16_t speed = 2;
 uint16_t scale = 75;
 
  uint16_t huecounter = 1;
 void start() {
  int direction = random(0, 2);
  if (direction == 0)
    direction = -1;
  
  for (int i = 0; i < NUM_PARTICLES; i++) {
    Boid boid = Boid(random(1,COLS-1), random(1,ROWS-1));
    boid.velocity.x = ((float) random(40, 50)) / 100.0;
    boid.velocity.x *= direction;
    boid.velocity.y = 0;
    boid.colorIndex = i * 32;
    boid.hue = huecounter;
    boid.id = i;
    huecounter += 0xFABCDE;
    boids[i] = boid;

  
}
    }
class Attractor {
  public:
    float mass; // Mass, tied to size
  float G; // Gravitational Constant
  PVector location; // Location
  
  Attractor() {
    location = PVector(320/2, 170/2);
    mass = 15;
    G = 0.9;
  }  
  PVector attract(Boid m) {
    PVector force = location - m.location; // Calculate direction of force
    float d = force.mag(); // Distance between objects
    d = constrain(d, 6.0, 10.0); // Limiting the distance to eliminate "extreme" results for very close or very far objects
    force.normalize(); 
  
  
  
  // Normalize vector (distance doesn't matter here, we just want this vector for direction)        
    float strength = (G * mass * m.mass) / (d * d); // Calculate gravitional force magnitude
    force *= strength; // Get force vector --> magnitude * direction
    return force;
    }
  };
Attractor attractor;
void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);  
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
void loop() {  
    if (loadingFlag)
    {
        loadingFlag = false;    
        start();     
    }     
  for (int i = 0; i < NUM_PARTICLES; i++) {
    Boid boid = boids[i];        
    tft.drawPixel(boid.location.y,boid.location.x,0x0000);
    //tft.drawCircle(boid.location.y,boid.location.x,lastboidsize[i],0x0000);
    //tft.fillCircle(boid.location.y,boid.location.x,lastboidsize[i],0x0000);
    PVector force = attractor.attract(boid);
    colors ++;
    PVector sep = boid.separate(boids, NUM_PARTICLES);   // Separation
    PVector ali = boid.align(boids, NUM_PARTICLES);      // Alignment
    PVector coh = boid.cohesion(boids, NUM_PARTICLES);   // Cohesion
    // Arbitrarily weight these forces
    sep *= 2.0;
    ali *= 1.0;
    coh *= 1.0;
    // Add the force vectors to acceleration
    boid.applyForce(sep);
    boid.applyForce(ali);
    boid.applyForce(coh);
    boid.applyForce(force);
    boid.update();
    difx[i] = boid.location.x - lastx[i];
    dify[i] = boid.location.y - lasty[i];      
    boid.size = 3 + boid.startingsize - ((difx[i] + dify[i]));
    if (boid.size <= 0)
    {
      boid.size = 0;
    }
    tft.drawPixel(boid.location.y,boid.location.x,boid.hue);
    //tft.drawCircle(boid.location.y,boid.location.x,boid.size,boid.hue);
    //tft.fillCircle(boid.location.y,boid.location.x,boid.size,boid.hue);
    lastboidsize[i] = boid.size;
    lastx[i] = boid.location.x;
    lasty[i] = boid.location.y;   
    boids[i] = boid;   
  }
}
