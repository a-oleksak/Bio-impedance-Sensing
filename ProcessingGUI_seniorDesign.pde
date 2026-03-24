// ==========================================================
// Skin Diagnosis GUI (Dual Heatmap)
// ==========================================================

import processing.serial.*;

Serial myPort;

// ---------------- USER PARAMETERS ----------------
int START_FREQ = 3000;
int FREQ_INC   = 1000;
int NUM_INCR   = 4;

int maxSteps = 4000/50 + 1;

// Two heatmaps
float[][] heatmapForward  = new float[NUM_INCR+1][maxSteps];
float[][] heatmapBackward = new float[NUM_INCR+1][maxSteps];

// Direction tracking
int prevStep = -1;
boolean forwardDirection = true;

// Layout
int leftMargin   = 100;
int rightMargin  = 300;
int topMargin    = 90;
int bottomMargin = 100;
int gapBetweenPlots = 120;

// Color Scale Constants
float Z_MIN = 40000;
float Z_MAX = 130000;

// interpolation resolution (performance control)
int INTERP_COLS = 200;
int INTERP_ROWS = 80;

void setup() {

  size(1800, 750);   
  frameRate(30);
  smooth(2);

  myPort = new Serial(this, Serial.list()[3], 115200);
  myPort.clear();
  myPort.bufferUntil('\n');
}

void draw() {

  background(245);

  float totalPlotW = width - leftMargin - rightMargin - gapBetweenPlots;
  float plotW = totalPlotW / 2.0;
  float plotH = height - topMargin - bottomMargin;

  // Forward heatmap
  drawHeatmap(heatmapForward,
              leftMargin,
              plotW,
              plotH,
              "Forward Scan");

  // Backward heatmap
  drawHeatmap(heatmapBackward,
              leftMargin + plotW + gapBetweenPlots,
              plotW,
              plotH,
              "Backward Scan");

  drawLegend();
  drawColorScale();
  drawCursorReadout();
}

  // Bilinear Interpolation Function
  float bilinear(float[][] data, float x, float y) {

    int x0 = floor(x);
    int x1 = min(x0 + 1, maxSteps - 1);

    int y0 = floor(y);
    int y1 = min(y0 + 1, NUM_INCR);

    float q11 = data[y0][x0];
    float q21 = data[y0][x1];
    float q12 = data[y1][x0];
    float q22 = data[y1][x1];

    float fx = x - x0;
    float fy = y - y0;

    float sum = 0;
    float weight = 0;

    if (q11 > 0) {
      float w = (1-fx)*(1-fy);
      sum += q11 * w;
      weight += w;
    }

    if (q21 > 0) {
      float w = fx*(1-fy);
      sum += q21 * w;
      weight += w;
    }

    if (q12 > 0) {
      float w = (1-fx)*fy;
      sum += q12 * w;
      weight += w;
    }

    if (q22 > 0) {
      float w = fx*fy;
      sum += q22 * w;
      weight += w;
    }

    if (weight == 0) return 0;

    return sum / weight;
  }

// ==========================================================
// HEATMAP DRAW FUNCTION
// ==========================================================

void drawHeatmap(float[][] mapData,
                 float xOffset,
                 float plotW,
                 float plotH,
                 String title) {



  // ---- Draw Cells ----
  // ---- FAST SMOOTH HEATMAP RENDERING ----

  float cellW = plotW / INTERP_COLS;
  float cellH = plotH / INTERP_ROWS;
  
// ---- X Axis Tick Marks ----

int maxMicro = 4000;   // total scan range
int smallStep = 50;    // small tick spacing
int bigStep   = 100;   // big tick spacing

for (int step = 0; step <= maxMicro; step += smallStep) {

  float x = map(step, 0, maxMicro, xOffset, xOffset + plotW);

  if (step % bigStep == 0) {
    strokeWeight(2); // big tick
    line(x,
         topMargin + plotH,
         x,
         topMargin + plotH + 10);
  } else {
    strokeWeight(1); // small tick
    line(x,
         topMargin + plotH,
         x,
         topMargin + plotH + 5);
  }
}

  noStroke();

  for (int iy = 0; iy < INTERP_ROWS; iy++) {

    for (int ix = 0; ix < INTERP_COLS; ix++) {

      float gx = map(ix, 0, INTERP_COLS-1, 0, maxSteps-1);
      float gy = map(iy, 0, INTERP_ROWS-1, 0, NUM_INCR);

      float z = bilinear(mapData, gx, gy);

      if (z <= 0) {

        fill(220);

      } else {

        float col = map(z, Z_MIN, Z_MAX, 255, 0);
        col = constrain(col, 0, 255);

        fill(col, 0, 255 - col);

      }

      rect(
        xOffset + ix * cellW,
        topMargin + iy * cellH,
        cellW + 1,
        cellH + 1
      );
    }
  }

  // ---- Bold Axes ----
  stroke(0);
  strokeWeight(1);

  line(xOffset, topMargin,
       xOffset, topMargin + plotH);

  line(xOffset, topMargin + plotH,
       xOffset + plotW, topMargin + plotH);

  // ---- Title ----
  fill(0);
  textAlign(CENTER);
  textSize(24);
  text(title, xOffset + plotW/2, 50);

  // ---- Y Labels ----
  textAlign(RIGHT, CENTER);
  textSize(18);

  float labelCellH = plotH / float(NUM_INCR+1);
  for (int r = 0; r < NUM_INCR+1; r++) {
    float y = topMargin + r * labelCellH + labelCellH/2;
    int freq = START_FREQ + r * FREQ_INC;
    text(freq/1000.0 + " kHz", xOffset - 7, y);
  }

// ---- X Labels ----
textAlign(CENTER, TOP);
textSize(16);

for (int step = 0; step <= maxMicro; step += 500) {
  float x = map(step, 0, maxMicro, xOffset, xOffset + plotW);
  text(step, x, topMargin + plotH + 15);
}

  // ---- Axis Titles ----
  textSize(20);
  text("Microsteps",
       xOffset + plotW/2,
       topMargin + plotH + 55);

  pushMatrix();
  translate(xOffset - 75, topMargin + plotH/2);
  rotate(-HALF_PI);
  textAlign(CENTER);
  text("Frequency (kHz)", 0, 0);
  popMatrix();

  textAlign(LEFT);
}

// ==========================================================
// LEGEND PANEL
// ==========================================================

void drawLegend() {

  int lx = width - rightMargin + 60;
  int ly = topMargin + 30;

  fill(0);
  textSize(20);
  text("Impedance Map Legend", lx, ly);

  textSize(16);

  // Low Z
  fill(255, 0, 0);
  rect(lx, ly + 25, 35, 18);
  fill(0);
  text("Low Z (Lesion)", lx + 50, ly + 40);

  // High Z
  fill(0, 0, 255);
  rect(lx, ly + 65, 35, 18);
  fill(0);
  text("High Z (Normal)", lx + 50, ly + 80);

  // No Data
  fill(220);
  rect(lx, ly + 105, 35, 18);
  fill(0);
  text("No Data", lx + 50, ly + 120);

  textSize(16);
  text("Press 'C' to clear scan", lx, ly + 170);
  text("Press 'S' to save screenshot", lx, ly + 205);
}

void drawColorScale() {

  int barW = 30;
  int barH = 320;

  // horizontal position (middle of right panel)
  int barX = width - rightMargin + 150;

  // vertical space calculation
  int legendBottom = topMargin + 300;   // bottom of legend block
  int freeSpaceTop = legendBottom;
  int freeSpaceBottom = height - 60;

  // center the bar
  int barY = (freeSpaceTop + freeSpaceBottom)/2 - barH/2 - 15; // slight upward shift

  // draw gradient
  for (int i = 0; i < barH; i++) {

    float z = map(i, 0, barH, Z_MAX, Z_MIN);

    float col = map(z, Z_MIN, Z_MAX, 255, 0);
    col = constrain(col,0,255);

    stroke(col,0,255-col);
    line(barX, barY + i, barX + barW, barY + i);
  }

  // border
  stroke(0);
  noFill();
  rect(barX, barY, barW, barH);

  // labels
  fill(0);
  textSize(14);
  textAlign(LEFT);

  text("Impedance", barX - 5, barY - 10);
  text(int(Z_MAX), barX + 40, barY + 5);
  text(int(Z_MIN), barX + 40, barY + barH);
}

void drawCursorReadout() {

  float totalPlotW = width - leftMargin - rightMargin - gapBetweenPlots;
  float plotW = totalPlotW / 2.0;
  float plotH = height - topMargin - bottomMargin;

  float cellW = plotW / float(maxSteps);
  float cellH = plotH / float(NUM_INCR+1);

  boolean inForward =
    mouseX > leftMargin &&
    mouseX < leftMargin + plotW;

  boolean inBackward =
    mouseX > leftMargin + plotW + gapBetweenPlots &&
    mouseX < leftMargin + 2*plotW + gapBetweenPlots;

  boolean inY =
    mouseY > topMargin &&
    mouseY < topMargin + plotH;

  if ((inForward || inBackward) && inY) {

    int col;
    float z;

    if (inForward) {

      col = int((mouseX - leftMargin) / cellW);

    } else {

      float backStart = leftMargin + plotW + gapBetweenPlots;
      col = int((mouseX - backStart) / cellW);

    }

    int row = int((mouseY - topMargin) / cellH);

    if (row>=0 && row<NUM_INCR+1 &&
        col>=0 && col<maxSteps) {

      if (inForward)
        z = heatmapForward[row][col];
      else
        z = heatmapBackward[row][col];

      int micro = col * 50;
      int freq  = START_FREQ + row * FREQ_INC;

      fill(0);
      textSize(16);
      textAlign(CENTER);

      String txt =
        "Freq: " + freq/1000.0 + " kHz   " +
        "Step: " + micro + "   " +
        "Z: " + int(z);

      text(txt, width/2, height - 40);
    }
  }
}

// ==========================================================
// SERIAL HANDLER
// ==========================================================

void serialEvent(Serial p) {

  String line = p.readStringUntil('\n');

  if (line == null) return;

  line = trim(line);
  String[] parts = split(line, ',');

  if (parts.length != 6) return;

  float freq = float(parts[0]);
  int step   = int(parts[1]);
  float zmag = float(parts[5]);

  if (Float.isNaN(zmag)) return;
  if(zmag <= 0) return;
  

  // Initialize first step
  if (prevStep == -1) {
    prevStep = step;
  }

  // Detect direction
  if (step > prevStep) forwardDirection = true;
  else if (step < prevStep) forwardDirection = false;

  prevStep = step;

  int row = int((freq/1000 - START_FREQ/1000) / (FREQ_INC/1000));
  int col = step/50;

  if (row>=0 && row<NUM_INCR+1 &&
      col>=0 && col<maxSteps) {

    if (forwardDirection) {
      heatmapForward[row][col] = zmag;
    } else {
      heatmapBackward[row][col] = zmag;
    }
  }
}

// ==========================================================
// CLEAR + SCREENSHOT
// ==========================================================

void keyPressed() {

  if (key == 'c' || key == 'C') {

    for (int r = 0; r < NUM_INCR+1; r++) {
      for (int c = 0; c < maxSteps; c++) {
        heatmapForward[r][c]  = 0;
        heatmapBackward[r][c] = 0;
      }
    }

    prevStep = -1;
  }

  if (key == 's' || key == 'S') {

    String filename = "scan-" + year() + "-" + month() + "-" + day() +
                      "-" + hour() + "-" + minute() + "-" + second() + ".png";

    saveFrame(filename);
    println("Screenshot saved: " + filename);
  }
}
