/**
 * Arduino Encoder Data Logger
 * Reads encoder data from Arduino Serial and saves to CSV
 * 
 * Before running:
 * 1. Upload Arduino code
 * 2. Close Arduino Serial Monitor
 * 3. Check Serial.list() output and update portIndex
 * 4. Run this sketch
 * 5. Press any key to stop and save
 */

import processing.serial.*;

Serial port;
PrintWriter out;

// CHANGE THIS to match your Arduino port
int portIndex = 0;  // <-- Change if needed

void setup() {
  size(400, 200);
  
  // List all available ports
  println("Available Serial Ports:");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println("[" + i + "] " + ports[i]);
  }
  
  // Connect to Arduino
  if (ports.length == 0) {
    println("ERROR: No serial ports found!");
    exit();
    return;
  }
  
  String portName = ports[portIndex];
  println("\nConnecting to: " + portName);
  port = new Serial(this, portName, 9600);
  port.bufferUntil('\n');
  
  // Create timestamped CSV file
  String fn = nf(year(),4)+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2);
  String filename = "encoder_log_"+fn+".csv";
  out = createWriter(filename);
  out.println("t_ms,encoderA,encoderB");
  
  println("Logging to: " + filename);
  println("Press ANY KEY to stop and save");
}

void draw() {
  background(50);
  fill(255);
  textAlign(CENTER, CENTER);
  text("Logging encoder data...\nPress ANY KEY to stop", width/2, height/2);
}

void serialEvent(Serial p) {
  String line = trim(p.readStringUntil('\n'));
  if (line == null || line.length() == 0) return;
  
  // Parse CSV: t_ms,encoderA,encoderB
  String[] parts = split(line, ',');
  if (parts.length == 3) {
    out.println(line);  // Write directly to file
    out.flush();        // Force write to disk
    
    // Optional: print to console for debugging
    println(line);
  }
}

void keyPressed() {
  println("\nStopping and saving...");
  out.flush();
  out.close();
  port.stop();
  exit();
}
