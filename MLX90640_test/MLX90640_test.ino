#include <Adafruit_MLX90640.h>

#define WIDTH  32
#define HEIGHT 24
#define RADIUS 8 // vertical distance to look at

Adafruit_MLX90640 mlx;
float frame[WIDTH*HEIGHT]; // buffer for full frame of temperatures


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("Serial working");

  if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found!");
  } else {
    Serial.println("Found Adafruit MLX90640");

    Serial.print("Serial number: ");
    Serial.print(mlx.serialNumber[0], HEX);
    Serial.print(mlx.serialNumber[1], HEX);
    Serial.println(mlx.serialNumber[2], HEX);
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_16BIT);
  mlx.setRefreshRate(MLX90640_16_HZ);
  Wire.setClock(1000000); // max 1 MHz
}

void loop() {
  // put your main code here, to run repeatedly:
  mlx.getFrame(frame);

  // Compress to 1D array by averaging
  int i;
  int n;
  float line[WIDTH];
  for(int col = 0; col < WIDTH; col++){
    n = 0;
    for(int row = HEIGHT/2 - RADIUS; row <= HEIGHT/2 + RADIUS; row++){
      i = WIDTH * row + col;
      if(!isnan(frame[i])){
        line[col] += frame[i];
        n++;
      }
    }
    line[col] = line[col] / n;
  }

  // Blur to smooth data
  float line_filt[WIDTH];
  line_filt[0] = 0.25 * line[0] + 0.5 * line[0] + 0.25 * line[1];
  for(int i = 1; i < WIDTH-1; i++){
    line_filt[i] = 0.25 * line[i] + 0.5 * line[i+1] + 0.25 * line[i+2];
  }
  line_filt[WIDTH-1] = 0.25 * line[WIDTH-2] + 0.5 * line[WIDTH-1] + 0.25 * line[WIDTH-1];

  // Find max temp location
  int iMax = 0;
  float max = 0;
  for(int i = 0; i < WIDTH; i++){
    if(line_filt[i] > max){
      max = line_filt[i];
      iMax = i;
    }
  }

  int error = iMax - WIDTH/2;

  for(int i = 0; i < WIDTH; i++){
    Serial.print(line_filt[i]);
    Serial.print(",");
  }
  Serial.println(iMax);

  /* for(int i = 0; i < 767; i++){
    Serial.print(frame[i]);
    Serial.print(",");
  }
  Serial.println(frame[767]); */
}
