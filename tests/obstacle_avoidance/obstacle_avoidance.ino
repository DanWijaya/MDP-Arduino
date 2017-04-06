void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
   Checklist *********************************************************************************
*/

void avoidObs() {
  double fl, fm, fr, lm;
  fl = final_MedianRead(irFL);
  fm = final_MedianRead(irFM);
  fr = final_MedianRead(irFR);
  //  Serial.print("1fl:");
  //  Serial.print(fl);
  //  Serial.print("   1fm:");
  //  Serial.print(fm);
  //  Serial.print("   1fr:");
  //  Serial.println(fr);
  while ((fl > 13 || fl < 8) && (fm > 13 || fm < 8) && (fr > 13 || fr < 8)) {
    forward(10.0);
    fl = final_MedianRead(irFL);
    fm = final_MedianRead(irFM);
    fr = final_MedianRead(irFR);
    //    Serial.print("fl:");
    //    Serial.print(fl);
    //    Serial.print("   fm:");
    //    Serial.print(fm);
    //    Serial.print("   fr:");
    //    Serial.println(fr);

    delay(50);
  }

  right(90.0);
  delay(50);

  forward(20.0);
  delay(50);
  left(90.0);


  lm = final_MedianRead(irLM);

  while (lm <= 16.5) {
    forward(10.0);
  }

  forward(10.0);
  left(90.0);
  forward(10.0);
}

void curved(double forward)
{
  double distanceL, distanceR, distanceTraversed, angular_error, v, w, setPoint;
  distanceTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  angular_error = 0;
  setPoint = 0;
  v = 110;
  w = 0;
  double Pi = 22 / 7;

  PID PID_angular(&angular_error, &w, &setPoint, angular_kp, angular_ki, angular_kd, DIRECT);
  PID_angular.SetMode(AUTOMATIC);

  while ( forward - distanceTraversed > 0.5) {
    distanceL += 2 * Pi * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * Pi * wheelRadius * en.getMotor2Revs();

    //    Serial.print("   DistanceL: ");
    //    Serial.print(distanceL/100);
    //    Serial.print(" ");
    //    Serial.print("   DistanceR: ");
    //    Serial.print(distanceR/100);
    //    Serial.print(" ");
    distanceTraversed = (distanceL + distanceR) / 2;

    angular_error = distanceL - distanceR;
    Serial.print("   Forward error: ");
    Serial.println(angular_error);
    //Serial.print(" ");
    //Serial.print("   Distance left: ");
    //Serial.println(distanceTraversed);

    PID_angular.Compute();
    if (fabs(forward - distanceTraversed) < forward / 2 && v > 180) {
      v = v - 0.0005 * v;
    }
    else if (v < 350) {
      v = v + 0.0005 * v;
    }
    w = w - 0.0005 * w;

    md.setSpeeds((-v - w) * 11 / 12, v - w);

  }

  v = fabs((-v - w) * 11 / 12) ;
  double cons_l = v;
  double  angleTraversed = 0;
  distanceL = 0;
  distanceR = 0;
  setPoint = 0;
  angular_error = 0;
  w = 0;
  md.setSpeeds(-cons_l / 1.5 , cons_l * 2.2 / 1.5);

  while (distanceL < 0.166 * Pi * 3 * distanceBetweenWheels / 2)
  {
    distanceL += 2 * Pi * wheelRadius * en.getMotor1Revs();
    distanceR += 2 * Pi * wheelRadius * en.getMotor2Revs();
    Serial.print("   distanceL ");
    Serial.print(distanceL);
    Serial.print(" ");

  }
  md.setBrakes(400, 400);
}

