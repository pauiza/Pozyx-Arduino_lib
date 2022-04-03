#include <UM7.h>
#include <TimerOne.h>
UM7 imu;
// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <SoftwareSerial.h>
////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////
uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x6847, 0x6822, 0x680E, 0x6851};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t heights[4] = {645, 463, 4577, 1242};                // anchor z-coordinates in mm
boolean bProcessing = false;                                // set this to true to output data for the processing sketch

// only required for manual anchor calibration. Please change this to the coordinates measured for the anchors
int32_t anchors_x[4] = {0, 9716, 4080, 12985};              // anchor x-coorindates in mm
int32_t anchors_y[4] = {0, 0, 14883, 10299};                  // anchor y-coordinates in mm

///////////////////////////////////////////////////Global Varible//
coordinates_t position;
SoftwareSerial mySerial(10, 11); // RX, TX
double x_sum=0;
double y_sum=0;
double z_sum=0;
float  yaw_sum=0;
float  yaw_rate=0;
double x_avg=0;
double y_avg=0;
double z_avg=0;
double x_avg_update=0;
double y_avg_update=0;
double z_avg_update=0;
double x_data[20];
double y_data[20];
double    x_var=0;
double    y_var=0;
int   x_counter=0;
int   y_counter=0;
int   z_counter=0;
int   i=0;    //array index
int   um7_counter=0;
volatile int channel=0;


void setup(){
  Pozyx.clearDevices();

  Serial.begin(115200);
  mySerial.begin(115200);
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start calibration"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing auto anchor calibration:"));
  Timer1.initialize(500000);
  Timer1.attachInterrupt(callback);

  
  int status = Pozyx.doAnchorCalibration(POZYX_2_5D, 10, num_anchors, anchors, heights);
  if (status != POZYX_SUCCESS){
    // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
    // fot this, you must update the arrays anchors_x, anchors_y and heights above
    // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
    SetAnchorsManual();
  }
  printCalibrationResult();
  delay(3000);
  Serial.println(F("Starting positioning: "));
}

void loop(){
  
  if(channel==0){
    int status = Pozyx.doPositioning(&position, POZYX_2_5D, 1000);
    pozyx_data_filtering(position);
  }
  if(channel==1){
    um7_read();
  }
}


void callback(){
  print_data();
  cleanup();
  switch_channel();
}
int verify_pozyx_data(double x_variance, double y_variance){
  if((x_variance<5)&&(y_variance<5)){
    x_avg=x_avg_update;
    y_avg=y_avg_update;
    z_avg=z_sum/z_counter;
    return 1;
    }else{
    Serial.println("Waiting for stablisation");
    return 0;
  }
}
void print_data(){
  if (channel==0){
    if(verify_pozyx_data(x_var, y_var)){
      if(!bProcessing){
        printCoordinates(x_avg,y_avg,z_avg);
        }else{
        printCoordinatesProcessing(x_avg,y_avg,z_avg);
      }
    }
  }
  if(channel==1){
    um7_print(yaw_sum/um7_counter,yaw_rate/um7_counter);
  }
}

void switch_channel(){
  if(channel==0){
    channel=1;
    }else{
    channel=0;
  }
}

void cleanup(){
  //pozyx data array
  if (channel==0){
    x_sum=0;y_sum=0;z_sum=0;
    memset(x_data,0,x_counter);memset(y_data,0,y_counter);
    x_counter=0;y_counter=0;z_counter=0;
    x_var=0;y_var=0;
    //sensor index counter
    i=0;
  }
  if (channel==1){
    //um7 data
    yaw_sum=0;yaw_rate=0;
    um7_counter=0;
  }
}
void pozyx_data_filtering(coordinates_t coor){
  //FOR POZYX SENSOR
  x_data[i]=coor.x/1000;
  x_sum+=coor.x/1000;
  y_data[i]=coor.y/1000;
  y_sum+=coor.y/1000;
  x_counter++;
  y_counter++;
  z_sum+=coor.z;
  z_counter++;
  for(int j=0;j<x_counter;j++){
    x_avg_update=x_sum/x_counter;
    x_var=(x_data[j]-x_avg)*(x_data[j]-x_avg)/(x_counter-1);
  }
  for(int k=0;k<y_counter;k++){
    y_avg_update=y_sum/y_counter;
    y_var=(y_data[k]-y_avg)*(y_data[k]-y_avg)/(y_counter-1);
  }
  i++;
}

void um7_read(){
  //for UM7 sensor
  if (mySerial.available() > 0) {
    if (imu.encode(mySerial.read())) {  // Reads byte from buffer.  Valid packet returns true.
      if(imu.address==0x70)
      {
        yaw_sum += imu.yaw/91.02222;
        yaw_rate += imu.yaw_rate/16.0;
        um7_counter++;
      }
    }
  }
}
void um7_print( float yaw, float yawrate){
  Serial.print("yaw: ");
  Serial.println(yaw);
  Serial.print("yaw rate: ");
  Serial.println(yawrate);
}

// function to print the coordinates to the serial monitor
void printCoordinates(double xvalue,double yvalue, double zvalue){
  
  Serial.print("x_mm: ");
  Serial.print(xvalue);
  Serial.print("\t");
  Serial.print("y_mm: ");
  Serial.print(yvalue);
  Serial.print("\t");
  Serial.print("z_mm: ");
  Serial.print(zvalue);
  Serial.println();
}


// function to print out positoining data + ranges for the processing sketch
void printCoordinatesProcessing(double xvalue,double yvalue, double zvalue){
  
  // get the network id and print it
  uint16_t network_id;
  Pozyx.getNetworkId(&network_id);
  
  Serial.print("POS,0x");
  Serial.print(network_id,HEX);
  Serial.print(",");
  Serial.print(xvalue);
  Serial.print(",");
  Serial.print(yvalue);
  Serial.print(",");
  Serial.print(zvalue);
  Serial.print(",");
  
  // get information about the positioning error and print it
  pos_error_t pos_error;
  Pozyx.getPositionError(&pos_error);
  
  Serial.print(pos_error.x);
  Serial.print(",");
  Serial.print(pos_error.y);
  Serial.print(",");
  Serial.print(pos_error.z);
  Serial.print(",");
  Serial.print(pos_error.xy);
  Serial.print(",");
  Serial.print(pos_error.xz);
  Serial.print(",");
  Serial.print(pos_error.yz);
  
  // read out the ranges to each anchor and print it
  for (int i=0; i < num_anchors; i++){
    device_range_t range;
    Pozyx.getDeviceRangeInfo(anchors[i], &range);
    Serial.print(",");
    Serial.print(range.distance);
    Serial.print(",");
    Serial.print(range.RSS);
  }
  Serial.println();
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list size: ");
  Serial.println(status*list_size);
  
  if(list_size == 0){
    Serial.println("Calibration failed.");
    Serial.println(Pozyx.getSystemError());
    return;
  }
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);
  
  coordinates_t anchor_coor;
  for(int i=0; i<list_size; i++)
  {
    
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
    
  }
}

// function to manually set the anchor coordinates
void SetAnchorsManual(){
  
  int i=0;
  for(i=0; i<num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor);
  }
  
}
