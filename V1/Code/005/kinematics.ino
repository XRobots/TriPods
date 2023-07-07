void kinematics (int leg, float xIn, float yIn, float zIn) {

    #define length1 0.095     // upper leg
    #define length2 0.165     // lower leg
    float angle1;
    float angle1a;
    float angle1b;
    float angle1c;
    float angle1Degrees;
    float angle2;
    float angle2a;
    float angle2b;
    float angle2c;
    float angle2Degrees;
    float angle3Degrees;     
    float z3;
    float shoulderAngle2;
    float shoulderAngle2Degrees;
    float shoulderAngleServo;
    float shoulderAngleServo2;
    float kneeAngleServo;

    float reach;
    float swing; 
    float swing1;
    float swingDegrees; 
    float swingDegrees2;
    float swingDegrees3;
    float swingServo1;
    float swingServo2;
    float swingServo3;

    float xIn2;
    float yIn2;
    float zIn2;

    if (leg == 1) {
      // calculate distance from leg swing point from cartesian coordinates
      
      if (xIn == 0) {
        xIn = 0.001;      // hack to avoid divide by zero
      }
      
      swing = atan(yIn / xIn);
      swingDegrees = swing * (180/PI);    // convert to degrees

      reach = sqrt(sq(yIn) + sq(xIn));  
      
      if (xIn > 0) {
        reach = reach *-1;  
      }
      
      // calculate angle of leg swing from cartesian coordinates
      swingServo1 = 1500 + (swingDegrees * 12.5);    // convert to servo clicks 
      

      Serial.print(xIn);
      Serial.print(" , ");
      Serial.print(yIn);
      Serial.print(" , ");
      Serial.print(swing);
      Serial.print(" , ");
      Serial.print(swingDegrees);
      Serial.print(" , ");
      Serial.println(reach);
      
    }

    if (leg == 2) {
        // calculate distance from leg swing point from cartesian coordinates
        
        swing = atan(xIn / yIn);
        swingDegrees = swing * (180/PI);    // convert to degrees
        
        reach = sqrt(sq(yIn) + sq(xIn));    
        // calculate angle of leg swing from cartesian coordinates

        if (yIn < 0) {
          swingDegrees2 = (90 - (swingDegrees * -1)) + 90;
        }
        else {
          swingDegrees2 = swingDegrees;
        }
        swingDegrees2 = swingDegrees2 - 45;   // remove default servo position
        swingServo2 = 1500 - (swingDegrees2 * 12.5);    // convert to servo clicks         
       
    }

    if (leg == 3) {

        yIn = yIn * -1;   // reverse axis
        
        // calculate distance from leg swing point from cartesian coordinates            
        
        swing = atan(xIn / yIn);        
        swingDegrees = swing * (180/PI);    // convert to degrees

        reach = sqrt(sq(yIn) + sq(xIn));    
        // calculate angle of leg swing from cartesian coordinates

        if (yIn <= 0) {
          swingDegrees2 = (90 - (swingDegrees * -1)) + 90;
        }
        else {
          swingDegrees2 = swingDegrees;
        }
        swingDegrees2 = swingDegrees2 - 45;   // remove default servo position
        swingServo3 = 1500 + (swingDegrees2 * 12.5);    // convert to servo clicks 
    }



    // calculate the shoulder joint offset and new leg length based on now far the foot moves in/out

    shoulderAngle2 = atan(reach/zIn);     // calc how much extra to add to the shoulder joint    
    z3 = zIn/cos(shoulderAngle2);     // calc new leg length to feed to the next bit of code below     

    // calculate leg length based on shin/thigh length and knee and shoulder angle
    
    angle1a = sq(length1) + sq(z3) - sq(length2);
    angle1b = 2 * length1 * z3;
    angle1c = angle1a / angle1b;
    angle1 = acos(angle1c);           // shoulder angle RADIANS

    angle2a = sq(z3) + sq(length2) - sq(length1);
    angle2b = 2 * z3 * length2;
    angle2c = angle2a / angle2b;
    angle2 = acos(angle2c);           // ankle angle RADIANS   

    //calc degrees from angles
    angle1Degrees = angle1 * (180/PI);    // shoulder angle DEGREES
    angle2Degrees = angle2 * (180/PI);    // ankle angle DEGREES
    angle3Degrees = 180 - angle1Degrees - angle2Degrees;   // knee angle DEGREES
    shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // front/back shoulder offset DEGREES


    // remove default position angles
    angle1Degrees = angle1Degrees - 47;
    angle3Degrees = angle3Degrees - 107;

    // turn into servo clicks

    shoulderAngleServo = 1500 - (angle1Degrees * 12.5);
    kneeAngleServo = 1500 - (angle3Degrees * 12.5);
    shoulderAngleServo2 = shoulderAngle2Degrees * 12.5;

    int factor = 10;

    if (leg == 1){ 

      pos1 = pos1Offset + swingServo1;
      pos4 = pos4Offset + shoulderAngleServo - shoulderAngleServo2;
      pos7 = pos7Offset + kneeAngleServo;
      pos1Filtered = filter(pos1,pos1Filtered,factor);
      pos4Filtered = filter(pos4,pos4Filtered,factor);
      pos7Filtered = filter(pos7,pos7Filtered,factor);
      servo1.writeMicroseconds(pos1Filtered);    
      servo4.writeMicroseconds(pos4Filtered);
      servo7.writeMicroseconds(pos7Filtered);
    }
    if (leg == 2){ 
      pos2 = pos2Offset + swingServo2;
      pos6 = pos6Offset + shoulderAngleServo - shoulderAngleServo2;
      pos9 = pos9Offset + kneeAngleServo;
      pos2Filtered = filter(pos2,pos2Filtered,factor);
      pos6Filtered = filter(pos6,pos6Filtered,factor);
      pos9Filtered = filter(pos9,pos9Filtered,factor);
      servo2.writeMicroseconds(pos2Filtered);
      servo6.writeMicroseconds(pos6Filtered);
      servo9.writeMicroseconds(pos9Filtered);
    }
    if (leg == 3){ 
      pos3 = pos3Offset + swingServo3;
      pos5 = pos5Offset + shoulderAngleServo - shoulderAngleServo2;
      pos8 = pos8Offset + kneeAngleServo;
      pos3Filtered = filter(pos3,pos3Filtered,factor);
      pos5Filtered = filter(pos5,pos5Filtered,factor);
      pos8Filtered = filter(pos8,pos8Filtered,factor);
      servo3.writeMicroseconds(pos3Filtered);
      servo5.writeMicroseconds(pos5Filtered);
      servo8.writeMicroseconds(pos8Filtered);
    }

     
}



