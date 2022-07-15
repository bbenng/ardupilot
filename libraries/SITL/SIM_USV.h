/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  usv simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a usv simulator
 */
class USV : public Aircraft {
public:
    USV(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new USV(frame_str);
    }

    bool on_ground() const override {return true;};

private:

    // return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
    float get_turn_circle(float steering) const;

    // return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
    float get_yaw_rate(float steering, float speed) const;

    // return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
    float get_lat_accel(float steering, float speed) const;

    // returns the heel angle due to turning in radians
    float get_heel_angle(float steering, float speed) const;

    // simulate waves and swell
    void update_wave(float delta_time);

    float turning_circle;            // vehicle minimum turning circle diameter in meters
    float turn_rate;                 // vehicle maximum turn rate in degrees/second

    // thrust curve based on Blue Robotics T200 thruster data at 16V
    // PWM values                     1100   1140   1180   1220   1260   1300   1340   1380   1420   1460  1500  1540  1580  1620  1660  1700  1740  1780  1820  1860  1900
    const float thrust_curve[21] = {-4.07f,-3.66f,-2.99f,-2.45f,-1.94f,-1.44f,-1.02f,-0.65f,-0.32f,-0.07f,0.00f,0.08f,0.40f,0.82f,1.28f,1.82f,2.43f,3.11f,3.82f,4.65f,5.25f}; // kgf
    
    float metacentric_height = 1.0f; // distance (GM) between center of gravity and metacentre (GM)
    float heeling_arm = 0.0f;        // perpendicular distance (BG) between the center of gravity and center of buoyancy
    const float mass = 6.0f;

    Vector3f velocity_ef_water;      // m/s
    Vector3f wave_gyro;              // rad/s
    float wave_heave;                // m/s/s
    float wave_phase;                // rads
};

} // namespace SITL