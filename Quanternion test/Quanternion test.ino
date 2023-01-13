#include <Geometry.h>


using namespace Geometry;
using namespace BLA;



void setup() {
  Serial.begin(9600);
  Translation a = {1,0,0};
  EulerAngles euler1(0, 0, M_PI_2);
  EulerAngles euler2(0, 0, 0);
  Pose p1,p2,p3;
  p1.p = a;
  p2.p = a;
  p1.R = euler1.to_rotation_matrix();
  p2.R = euler2.to_rotation_matrix();
  Serial << p1 << "\n";
  Serial << p2 << "\n";
  p3 = p1 * p2;
  Serial << p3 << "\n";
}

void loop() {
  // put your main code here, to run repeatedly:

}
