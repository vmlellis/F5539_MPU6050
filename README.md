```
//******************************************************************************
//  MSP430F5529 Demo - MPU6050
//
//  This demo connects the MSP430 and MPU6050 via the I2C bus. 
//  The master transmits to the slave. This is the MASTER CODE.
//  Check if MPU6050 is enaled.
//
//
//                                /|\  /|\
//                  MPU6050       10k  10k      MSP430F5529
//                   slave         |    |         master
//             -----------------   |    |   -----------------
//           -|              SDA|<-|----+->|P3.0/UCB0SDA  XIN|-
//            |                 |  |       |                 |
//           -|                 |  |       |             XOUT|-
//            |              SCL|<-+------>|P3.1/UCB0SCL     |
//            |                 |          |                 |
//
//   Victor Lellis
//   March 2015
//   Built with CCSv6 with GNU v4.9.1 (Red Hat)
//******************************************************************************
