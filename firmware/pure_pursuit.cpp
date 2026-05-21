#include "pure_pursuit.h"
#include <math.h>

static float wrap_pi(float a){
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

bool pp_compute_angle(const Pose& r, const Pose& t,
                      const PurePursuitParams& P,
                      int& outAngleDeg,
                      float& outDistM)
{
  if (t.stamp_ms == 0) return false;

  float dx   = t.x - r.x;
  float dy   = t.y - r.y;
  float dist = sqrtf(dx*dx + dy*dy);
  outDistM = dist;

  float bearing = atan2f(dy, dx);
  float alpha   = wrap_pi(bearing - r.yaw);

  float Ld = dist;
  if (Ld < P.min_lookahead_m) Ld = P.min_lookahead_m;
  if (Ld > P.max_lookahead_m) Ld = P.max_lookahead_m;

  float num   = 2.0f * P.wheelbase_m * sinf(alpha);
  float delta = atanf(num / Ld);

  const float maxSteerRad = P.max_steer_deg * (M_PI / 180.0f);
  if (delta >  maxSteerRad) delta =  maxSteerRad;
  if (delta < -maxSteerRad) delta = -maxSteerRad;

  float norm = delta / maxSteerRad; // [-1,1]
  const int leftSpan  = max(1, P.servo_center_deg - P.servo_min_deg);
  const int rightSpan = max(1, P.servo_max_deg - P.servo_center_deg);
  const float span = (norm < 0.0f) ? (float)leftSpan : (float)rightSpan;
  int angle = (int)roundf((float)P.servo_center_deg + norm * span);

  // Safety clamp to the configured servo range
  if (angle < P.servo_min_deg) angle = P.servo_min_deg;
  if (angle > P.servo_max_deg) angle = P.servo_max_deg;

  outAngleDeg = angle;
  return true;
}


