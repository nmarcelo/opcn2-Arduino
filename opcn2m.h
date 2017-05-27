/*
    opcn2m.h - Functions to operate the Alphasense OPC-N2 Particle counter..
    Created by David H Hagan, March 2016.
    Modified by Marcelo Yungaicela, May 2017
    Released with an MIT license.
*/
#include <SPI.h>
#include "opcn2.h"

OPCN2::OPCN2(uint8_t chip_select)
{
  // Initiate an instance of the OPCN2 class
  // Ex. OPCN2 alpha(chip_select = A2);
  _CS = chip_select;
 
 pinMode(_CS, OUTPUT);
  // Set up SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  // Set the firmware version
  firm_ver.major = 18;
  firm_ver.minor = 2;
  
}

uint16_t OPCN2::_16bit_int(byte LSB, byte MSB)
{
  // Combine two bytes into a 16-bit unsigned int
  return ((MSB << 8) | LSB);
}

bool OPCN2::_compare_arrays(byte array1[], byte array2[], int length)
{
  // Compare two arrays and return a boolean
  bool result = true;

  for (int i = 0; i < length; i++){
    if (array1[i] != array2[i]){
      result = false;
    }
  }

  return result;
}

float OPCN2::_calculate_float(byte val0, byte val1, byte val2, byte val3)
{
  // Return an IEEE754 float from an array of 4 bytes
  union u_tag {
    byte b[4];
    float val;
  } u;

  u.b[0] = val0;
  u.b[1] = val1;
  u.b[2] = val2;
  u.b[3] = val3;

  return u.val;
}

uint32_t OPCN2::_32bit_int(byte val0, byte val1, byte val2, byte val3)
{
  // Return a 32-bit unsigned int from 4 bytes
  //return ((val3 << 24) | (val2 << 16) | (val1 << 8) | val0);
  
  uint32_t val  = 0;
  val = val + val0;
  val << 8;
  val = val + val1;
  val << 8;
  val = val + val2;
  val << 8;
  val = val + val3;

  return val;
}

bool OPCN2::ping()
{
  // Isse the check status command
  // ex.
  // $ alpha.ping();
  // $ true
  byte resp[1];
  byte expected[] = {0xF3};

  digitalWrite(this->_CS, LOW);       // pull the pin low
  resp[0] = SPI.transfer(0xCF);       // issue the command byte
  digitalWrite(this->_CS, HIGH);      // pull the pin high

  return this->_compare_arrays(resp, expected, 1);
}

bool OPCN2::on()
{
  // Turn ON the OPC and return a boolean
  // Ex.
  // $ alpha.on()
  // $ true
  byte vals[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  vals[0] = SPI.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  vals[1] = SPI.transfer(0x00);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(vals, expected, 2);
}

bool OPCN2::off()
{
  // Turn OFF the OPC and return a boolean
  // Ex.
  // $ alpha.off()
  // $ true
  byte vals[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  vals[0] = SPI.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  vals[1] = SPI.transfer(0x01);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(vals, expected, 2);
}

String OPCN2::read_information_string()
{
  // Read the information string and return a string
  // Ex.
  // $ alpha.read_information_string()
  String result = "";
  String tmp;
  byte vals[61];

  digitalWrite(this->_CS, LOW);
  SPI.transfer(0x3F);
  digitalWrite(this->_CS, HIGH);

  delay(3);

  // Iterate to read the entire string
  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 60; i++){
    vals[i] = SPI.transfer(0x00);
    result += String((char)vals[i]);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);

  return result;
}

struct Status OPCN2::read_status()
{
  // Read key status variables from the OPC-N2
  // Only available with Alphasense OPC-N2 > firmware v18
  // Ex.
  // $ alpha.read_status()
  Status data;
  byte vals[4];

    // Read the status
    digitalWrite(this->_CS, LOW);
    SPI.transfer(0x13);
    digitalWrite(this->_CS, HIGH);

    delay(10);

    // Send the read command and build the array of data
    digitalWrite(this->_CS, LOW);
    for (int i = 0; i < 4; i++){
      vals[i] = SPI.transfer(0x13);
      delayMicroseconds(4);

    digitalWrite(this->_CS, HIGH);

    // Calculate the values!
    data.fanON    = (unsigned int)vals[0];
    data.laserON  = (unsigned int)vals[1];
    data.fanDAC   = (unsigned int)vals[2];
    data.laserDAC = (unsigned int)vals[3];
  }

  return data;
}

struct Firmware OPCN2::read_firmware_version()
{
  // Only available with Alphasense OPC-N2 > firmware v18
  // Ex.
  // $ alpha.read_firmware_version()
  Firmware res;
    // Read the Firmware version
    digitalWrite(this->_CS, LOW);
    SPI.transfer(0x12);
    digitalWrite(this->_CS, HIGH);

    delay(10);

    digitalWrite(this->_CS, LOW);
    res.major = (unsigned int)SPI.transfer(0x00);
    delayMicroseconds(4);
    res.minor = (unsigned int)SPI.transfer(0x00);
    digitalWrite(this->_CS, HIGH);
    return res;
}

bool OPCN2::write_config_variables(byte values[])
{
  // Write the configuration [NOT IMPLEMENTED]
  return true;
}

bool OPCN2::write_config_variables2(byte values[])
{
  // Write the configuration [NOT IMPLEMENTED]
  // Only available with Alphasense OPC-N2 > firmware v18
  return true;
}

bool OPCN2::write_serial_number_string(byte values[])
{
  // NOT IMPLEMENTED
  // Only available with Alphasense OPC-N2 > firmware v18
  return true;
}

bool OPCN2::save_config_variables()
{
  // Save the configuration variables
  // Ex.
  // $ alpha.save_config_variables()
  byte resp[6];
  byte commands[] = {0x43, 0x3F, 0x3C, 0x3F, 0x3C, 0x43};
  byte expected[] = {0xF3, 0x43, 0x3f, 0x3c, 0x3f, 0x3c};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI.transfer(commands[0]);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  for (int i = 1; i < (int)sizeof(commands); i++){
    resp[i] = SPI.transfer(commands[i]);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);      // Pull the pin high

  return this->_compare_arrays(resp, expected, 6);
}

bool OPCN2::enter_bootloader()
{
  // Enter into bootloader mode
  byte resp[1];
  byte expected[] = {0xF3};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI.transfer(0x41);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 1);
}

bool OPCN2::set_fan_power(uint8_t value)
{
  // Set the fan power
  // Value must be between 0-255
  // Ex.
  // $ alpha.set_fan_power(255);
  byte resp[3];
  byte expected[] = {0xF3, 0x42, 0x00};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI.transfer(0x42);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  resp[1] = SPI.transfer(0x00);

  delayMicroseconds(4);

  resp[2] = SPI.transfer(value);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 3);
}

bool OPCN2::set_laser_power(uint8_t value)
{
  // Set the laser power
  // Value must be between 0-255
  // Ex.
  // $ alpha.set_laser_power(255);
  byte resp[3];
  byte expected[] = {0xF3, 0x42, 0x01};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI.transfer(0x42);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  resp[1] = SPI.transfer(0x01);

  delayMicroseconds(4);

  resp[2] = SPI.transfer(value);
  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 3);
}

bool OPCN2::toggle_fan(bool state)
{
  // Toggle the state of the fan
  // Ex.
  // $ alpha.toggle_fan(true); // turns fan on
  byte resp[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  // turn either on or off
  digitalWrite(this->_CS, LOW);
  if (state == true){
    resp[1] = SPI.transfer(0x04);
  }
  else {
    resp[1] = SPI.transfer(0x05);
  }

  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 2);
}

bool OPCN2::toggle_laser(bool state)
{
  // Toggle the state of the laser
  // Ex.
  // $ alpha.toggle_laser(true);
  byte resp[2];
  byte expected[] = {0xF3, 0x03};

  digitalWrite(this->_CS, LOW);
  resp[0] = SPI.transfer(0x03);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  if (state == true){
    resp[1] = SPI.transfer(0x02);
  }
  else {
    resp[1] = SPI.transfer(0x03);
  }

  digitalWrite(this->_CS, HIGH);

  return this->_compare_arrays(resp, expected, 2);
}

struct ConfigVars OPCN2::read_configuration_variables()
{
  // Read the configuration variables and return the structure
  // Ex.
  // $ alpha.read_configuration_variables();
  ConfigVars results;       // empty structure for the data
  byte vals[256];

  // Read the config variables
  digitalWrite(this->_CS, LOW);
  SPI.transfer(0x3c);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 256; i++){
    vals[i] = SPI.transfer(0x00);
    delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);

  // Fill in the results
  results.bb0 = this->_16bit_int(vals[0], vals[1]);
  results.bb1 = this->_16bit_int(vals[2], vals[3]);
  results.bb2 = this->_16bit_int(vals[4], vals[5]);
  results.bb3 = this->_16bit_int(vals[6], vals[7]);
  results.bb4 = this->_16bit_int(vals[8], vals[9]);
  results.bb5 = this->_16bit_int(vals[10], vals[11]);
  results.bb6 = this->_16bit_int(vals[12], vals[13]);
  results.bb7 = this->_16bit_int(vals[14], vals[15]);
  results.bb8 = this->_16bit_int(vals[16], vals[17]);
  results.bb9 = this->_16bit_int(vals[18], vals[19]);
  results.bb10 = this->_16bit_int(vals[20], vals[21]);
  results.bb11 = this->_16bit_int(vals[22], vals[23]);
  results.bb12 = this->_16bit_int(vals[24], vals[25]);
  results.bb13 = this->_16bit_int(vals[26], vals[27]);
  results.bb14 = this->_16bit_int(vals[28], vals[29]);

  // Bin Particle Volumes
  results.bpv0 = this->_calculate_float(vals[32], vals[33], vals[34], vals[35]);
  results.bpv1 = this->_calculate_float(vals[36], vals[37], vals[38], vals[39]);
  results.bpv2 = this->_calculate_float(vals[40], vals[41], vals[42], vals[43]);
  results.bpv3 = this->_calculate_float(vals[44], vals[45], vals[46], vals[47]);
  results.bpv4 = this->_calculate_float(vals[48], vals[49], vals[50], vals[51]);
  results.bpv5 = this->_calculate_float(vals[52], vals[53], vals[54], vals[55]);
  results.bpv6 = this->_calculate_float(vals[56], vals[57], vals[58], vals[59]);
  results.bpv7 = this->_calculate_float(vals[60], vals[61], vals[62], vals[63]);
  results.bpv8 = this->_calculate_float(vals[64], vals[65], vals[66], vals[67]);
  results.bpv9 = this->_calculate_float(vals[68], vals[69], vals[70], vals[71]);
  results.bpv10 = this->_calculate_float(vals[72], vals[73], vals[74], vals[75]);
  results.bpv11 = this->_calculate_float(vals[76], vals[77], vals[78], vals[79]);
  results.bpv12 = this->_calculate_float(vals[80], vals[81], vals[82], vals[83]);
  results.bpv13 = this->_calculate_float(vals[84], vals[85], vals[86], vals[87]);
  results.bpv14 = this->_calculate_float(vals[88], vals[89], vals[90], vals[91]);
  results.bpv15 = this->_calculate_float(vals[92], vals[93], vals[94], vals[95]);

  // Bin Particle Densities
  results.bpd0 = this->_calculate_float(vals[96], vals[97], vals[98], vals[99]);
  results.bpd1 = this->_calculate_float(vals[100], vals[101], vals[102], vals[103]);
  results.bpd2 = this->_calculate_float(vals[104], vals[105], vals[106], vals[107]);
  results.bpd3 = this->_calculate_float(vals[108], vals[109], vals[110], vals[111]);
  results.bpd4 = this->_calculate_float(vals[112], vals[113], vals[114], vals[115]);
  results.bpd5 = this->_calculate_float(vals[116], vals[117], vals[118], vals[119]);
  results.bpd6 = this->_calculate_float(vals[120], vals[121], vals[122], vals[123]);
  results.bpd7 = this->_calculate_float(vals[124], vals[125], vals[126], vals[127]);
  results.bpd8 = this->_calculate_float(vals[128], vals[129], vals[130], vals[131]);
  results.bpd9 = this->_calculate_float(vals[132], vals[133], vals[134], vals[135]);
  results.bpd10 = this->_calculate_float(vals[136], vals[137], vals[138], vals[139]);
  results.bpd11 = this->_calculate_float(vals[140], vals[141], vals[142], vals[143]);
  results.bpd12 = this->_calculate_float(vals[144], vals[145], vals[146], vals[147]);
  results.bpd13 = this->_calculate_float(vals[148], vals[149], vals[150], vals[151]);
  results.bpd14 = this->_calculate_float(vals[152], vals[153], vals[154], vals[155]);
  results.bpd15 = this->_calculate_float(vals[156], vals[157], vals[158], vals[159]);

  // Bin Particle Sample Volumes
  results.bsvw0 = this->_calculate_float(vals[160], vals[161], vals[162], vals[163]);
  results.bsvw1 = this->_calculate_float(vals[164], vals[165], vals[166], vals[167]);
  results.bsvw2 = this->_calculate_float(vals[168], vals[169], vals[170], vals[171]);
  results.bsvw3 = this->_calculate_float(vals[172], vals[173], vals[174], vals[175]);
  results.bsvw4 = this->_calculate_float(vals[176], vals[177], vals[178], vals[179]);
  results.bsvw5 = this->_calculate_float(vals[180], vals[181], vals[182], vals[183]);
  results.bsvw6 = this->_calculate_float(vals[184], vals[185], vals[186], vals[187]);
  results.bsvw7 = this->_calculate_float(vals[188], vals[189], vals[190], vals[191]);
  results.bsvw8 = this->_calculate_float(vals[192], vals[193], vals[194], vals[195]);
  results.bsvw9 = this->_calculate_float(vals[196], vals[197], vals[198], vals[199]);
  results.bsvw10 = this->_calculate_float(vals[200], vals[201], vals[202], vals[203]);
  results.bsvw11 = this->_calculate_float(vals[204], vals[205], vals[206], vals[207]);
  results.bsvw12 = this->_calculate_float(vals[208], vals[209], vals[210], vals[211]);
  results.bsvw13 = this->_calculate_float(vals[212], vals[213], vals[214], vals[215]);
  results.bsvw14 = this->_calculate_float(vals[216], vals[217], vals[218], vals[219]);
  results.bsvw15 = this->_calculate_float(vals[220], vals[221], vals[222], vals[223]);

  // Gain Scaling Coefficient
  results.gsc = this->_calculate_float(vals[224], vals[225], vals[226], vals[227]);

  // Sample Flow Rate (ml/s)
  results.sfr = this->_calculate_float(vals[228], vals[229], vals[230], vals[231]);

  // LaserDAC
  results.laser_dac = (unsigned int)vals[232];
  results.fan_dac = (unsigned int)vals[233];

  // Time-of-Flight to Sample Flow Rate ratio
  results.tof_sfr = (unsigned int)vals[234];

  return results;
}

struct ConfigVars2 OPCN2::read_configuration_variables2()
{
  // Read the additional configuration variables
  // Only available on OPCN2's with firmware >v18
  // Ex.
  // $ alpha.read_configuration_variables2();
  ConfigVars2 results;       // empty structure for the data
  byte vals[9];

   // Read the config variables
  digitalWrite(this->_CS, LOW);
  SPI.transfer(0x3D);
  digitalWrite(this->_CS, HIGH);

  delay(10);

  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 9; i++){
      vals[i] = SPI.transfer(0x00);
      delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);

  // Fill in the results
  results.AMSamplingInterval    = this->_16bit_int(vals[0], vals[1]);
  results.AMIntervalCount       = this->_16bit_int(vals[2], vals[3]);
  results.AMFanOnIdle           = (unsigned int)vals[4];
  results.AMLaserOnIdle         = (unsigned int)vals[5];
  results.AMMaxDataArraysInFile = this->_16bit_int(vals[6], vals[7]);
  results.AMOnlySavePMData      = (unsigned int)vals[8];

return results;
}

String OPCN2::read_serial_number()
{
  // Read the serial number of the OPC
  // Only available on OPCN2's with firmware >v18
  // Ex.
  // $ alpha.read_serial_number();
  String result = "";
  byte vals[60];

    digitalWrite(this->_CS, LOW);       // Pull the CS low
    SPI.transfer(0x10);                 // Send the start command
    digitalWrite(this->_CS, HIGH);       // Pull the CS High

    delay(3);

    // Iterate to read the entire string
    digitalWrite(this->_CS, LOW);
    for (int i = 0; i < 60; i++){
        vals[i] = SPI.transfer(0x00);
        result += String((char)vals[i]);
        delayMicroseconds(4);
    }

   digitalWrite(this->_CS, HIGH);
  
   result.trim();

  return result;
}

struct PMData OPCN2::read_pm_data()
{
  // Read the PM Data and reset the histogram, return the struct
  // Only available on OPCN2's with firmware >v18
  // Ex.
  // $ alpha.read_pm_data();
  PMData data;
  byte vals[12];
  // Read the data and clear the local memory
  digitalWrite(this->_CS, LOW);       // Pull the CS Low
  SPI.transfer(0x32);                 // Transfer the command byte
  digitalWrite(this->_CS, HIGH);

  delay(12);                          // Delay for 12 ms

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);

  for (int i = 0; i < 12; i++){
      vals[i] = SPI.transfer(0x00);
      delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);      // Pull the CS High

  data.pm1  = this->_calculate_float(vals[0], vals[1], vals[2], vals[3]);
  data.pm25 = this->_calculate_float(vals[4], vals[5], vals[6], vals[7]);
  data.pm10 = this->_calculate_float(vals[8], vals[9], vals[10], vals[11]);

  return data;
}

struct HistogramData OPCN2::read_histogram(bool convert_to_conc)
{
  // Read the Histogram Data and reset the histogram, return the struct
  // convert_to_conc can be set to true if you would like the result
  // returned as concentrations (rather than raw counts) with units of
  // particles per cubic centimeter [#/cc]
  // Ex.
  // $ alpha.read_histogram(true)
  HistogramData data;
  byte vals[62];

  // Read the data and clear the local memory
  digitalWrite(this->_CS, LOW);       // Pull the CS Low
  SPI.transfer(0x30);                 // Transfer the command byte
  digitalWrite(this->_CS, HIGH);

  delay(12);                          // Delay for 12 ms

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);

  for (int i = 0; i < 62; i++){
      vals[i] = SPI.transfer(0x00);
      delayMicroseconds(4);
  }

  digitalWrite(this->_CS, HIGH);      // Pull the CS High

  data.period = this->_calculate_float(vals[44], vals[45], vals[46], vals[47]);
  data.sfr    = this->_calculate_float(vals[36], vals[37], vals[38], vals[39]);

  // If convert_to_conc = True, convert from raw data to concentration
  double conv;

  if ( convert_to_conc != true ) {
      conv = 1.0;
  }
  else {
      conv = data.sfr * data.period;
  }

  // Calculate all of the values!
  data.bin0   = (double)this->_16bit_int(vals[0], vals[1]) / conv;
  data.bin1   = (double)this->_16bit_int(vals[2], vals[3]) / conv;
  data.bin2   = (double)this->_16bit_int(vals[4], vals[5]) / conv;
  data.bin3   = (double)this->_16bit_int(vals[6], vals[7]) / conv;
  data.bin4   = (double)this->_16bit_int(vals[8], vals[9]) / conv;
  data.bin5   = (double)this->_16bit_int(vals[10], vals[11]) / conv;
  data.bin6   = (double)this->_16bit_int(vals[12], vals[13]) / conv;
  data.bin7   = (double)this->_16bit_int(vals[14], vals[15]) / conv;
  data.bin8   = (double)this->_16bit_int(vals[16], vals[17]) / conv;
  data.bin9   = (double)this->_16bit_int(vals[18], vals[19]) / conv;
  data.bin10  = (double)this->_16bit_int(vals[20], vals[21]) / conv;
  data.bin11  = (double)this->_16bit_int(vals[22], vals[23]) / conv;
  data.bin12  = (double)this->_16bit_int(vals[24], vals[25]) / conv;
  data.bin13  = (double)this->_16bit_int(vals[26], vals[27]) / conv;
  data.bin14  = (double)this->_16bit_int(vals[28], vals[29]) / conv;
  data.bin15  = (double)this->_16bit_int(vals[30], vals[31]) / conv;

  data.bin1MToF = int(vals[32]) / 3.0;
  data.bin3MToF = int(vals[33]) / 3.0;
  data.bin5MToF = int(vals[34]) / 3.0;
  data.bin7MToF = int(vals[35]) / 3.0;

  // This holds either temperature or pressure
  // If temp, this is temp in C x 10
  // If pressure, this is pressure in Pa
  data.temp_pressure = this->_32bit_int(vals[40], vals[41], vals[42], vals[43]);

  data.checksum = this->_16bit_int(vals[48], vals[49]);

  data.pm1 = this->_calculate_float(vals[50], vals[51], vals[52], vals[53]);
  data.pm25 = this->_calculate_float(vals[54], vals[55], vals[56], vals[57]);
  data.pm10 = this->_calculate_float(vals[58], vals[59], vals[60], vals[61]);

  return data;
}
