#!/usr/bin/env python3
"""Execute production allocation, rotation and PID integration with host stubs.

No flight dynamics, output protocol, thrust curve, or sensor model is simulated.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root/'ardupilot/libraries/AP_Motors/AP_MotorsMatrix_6DoF_Scripting.cpp').read_text()
function = source[source.index('void AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()'):source.index('// sets the roll and pitch offset')]
pid = (root/'ardupilot/libraries/AC_PID/AC_PID.cpp').read_text()
pid = pid[pid.index('void AC_PID::update_i('):pid.index('float AC_PID::get_p()')]
matrix = (root/'ardupilot/libraries/AP_Math/matrix3.cpp').read_text()
matrix = matrix[matrix.index('void Matrix3<T>::from_euler312('):matrix.index('// apply an additional rotation')]
matrix = matrix.replace('Matrix3<T>', 'Matrix3f').replace('T ', 'float ').replace('cosF(', 'std::cos(').replace('sinF(', 'std::sin(')
rows = []
for match in re.finditer(r'Motors_6DoF:add_motor\((.*?)\)', (root/'scripts/base_drone.lua').read_text()):
    args = match[1].split(',')
    rows.append('{' + ','.join(args[1:7]) + '}')
harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#define AP_MOTORS_MAX_NUM_MOTORS 8
#define MIN(a,b) std::min(a,b)
float constrain_float(float v,float lo,float hi) { return std::clamp(v,lo,hi); }
bool is_positive(float v) { return v>0; }
bool is_negative(float v) { return v<0; }
bool is_zero(float v) { return std::abs(v)<1e-7f; }
struct Vector3f { float x,y,z; };
struct Matrix3f {
    Vector3f a,b,c;
    void from_euler312(float r,float p,float y);
    Vector3f operator*(Vector3f v) {
        return {a.x*v.x+a.y*v.y+a.z*v.z,b.x*v.x+b.y*v.y+b.z*v.z,c.x*v.x+c.y*v.y+c.z*v.z};
    }
};
struct Param { float get() { return 200; } };
struct AC_PID {
    float _ki=.135f, _kimax=.5f, _integrator=0, _error=.0735f;
    void update_i(float dt,bool limit,float i_scale);
};
struct AP_MotorsMatrix_6DoF_Scripting {
    bool motor_enabled[8]={true,true,true,true,true,true,true,true};
    bool _reversible[8]={};
    float _roll_factor[8],_pitch_factor[8],_yaw_factor[8],_throttle_factor[8],_forward_factor[8],_right_factor[8];
    float _thrust_rpyt_out[8]={},_last_thrust_out[8]={};
    float _roll_in=0,_pitch_in=0,_yaw_in=0,_roll_in_ff=0,_pitch_in_ff=0,_yaw_in_ff=0;
    float throttle=.2f,forward=0,lateral=0,_roll_offset=0,_pitch_offset=0;
    struct { float get_compensation_gain() { return 1; } } thr_lin;
    struct Limits {
        bool roll=false,pitch=false,yaw=false,throttle_lower=false,throttle_upper=false;
        void set_rpy(bool b) { roll=pitch=yaw=b; }
    } limit;
    Param _yaw_headroom;
    float get_throttle() { return throttle; }
    float get_forward() { return forward; }
    float get_lateral() { return lateral; }
    void output_armed_stabilizing();
    AP_MotorsMatrix_6DoF_Scripting();
    void run() { limit={}; output_armed_stabilizing(); }
};
'''
init = 'const float factors[8][6]={' + ','.join(rows) + '};\n' + r'''
AP_MotorsMatrix_6DoF_Scripting::AP_MotorsMatrix_6DoF_Scripting() {
    for(int i=0;i<8;i++) {
        _roll_factor[i]=factors[i][0]; _pitch_factor[i]=factors[i][1];
        _yaw_factor[i]=factors[i][2]; _throttle_factor[i]=factors[i][3];
        _forward_factor[i]=factors[i][4]; _right_factor[i]=factors[i][5];
    }
}
'''
checks = r'''
bool close(float a,float b) { return std::abs(a-b)<2e-6f; }
int main() {
    using Mixer=AP_MotorsMatrix_6DoF_Scripting;
    // Low-throttle roll and pitch, both directions: no negative lift, flag limits.
    for(int axis=0;axis<2;axis++) for(float sign : {-1.f,1.f}) {
        Mixer m; m.throttle=.01f;
        (axis==0 ? m._roll_in : m._pitch_in)=sign*.08f; m.run();
        assert(m.limit.roll && m.limit.pitch);
        float sum=0;
        for(int i=0;i<8;i+=2) { assert(m._thrust_rpyt_out[i]>=0); sum+=m._thrust_rpyt_out[i]; }
        assert(close(sum, .01f*(2+2*factors[4][3])));
    }
    // Below saturation, preserve the existing allocation including signed lateral selection.
    for(float right : {-.1f,0.f,.1f}) for(float yaw : {-.03f,0.f,.03f}) {
        Mixer m; m.throttle=.3f; m.lateral=right; m._yaw_in=yaw;
        m._roll_in=.01f; m._pitch_in=-.02f; m.run();
        assert(!m.limit.roll);
        for(int i=0;i<8;i++) {
            float expected=.01f*factors[i][0]-.02f*factors[i][1]+yaw*factors[i][2]+.3f*factors[i][3]+.3f*right*factors[i][5];
            assert(close(expected,m._thrust_rpyt_out[i]));
        }
    }
    // Translation must not subtract more lift than remains after attitude correction.
    Mixer h; h.throttle=.02f; h._roll_in=.019f; h.lateral=1; h.run();
    for(int i=0;i<8;i+=2) assert(h._thrust_rpyt_out[i]>=0);
    // Production integral update receives previous-cycle allocation limits (400 Hz).
    Mixer m; AC_PID p; m.throttle=.01f;
    for(int n=0;n<4000;n++) {
        p.update_i(.0025f,m.limit.roll,1);
        m._roll_in=.135f*p._error+p._integrator; m.run();
    }
    assert(m.limit.roll && p._integrator<.001f);
    p._integrator=.02f; p._error=-.0735f;
    p.update_i(.0025f,true,1); assert(p._integrator<.02f); // can unwind
    Mixer z; z.throttle=0; z._roll_in=.1f; z.run();
    assert(z.limit.roll); for(int i=0;i<8;i+=2) assert(z._thrust_rpyt_out[i]==0);
    // Reversible motors retain signed lower limits.
    Mixer r; for(int i=0;i<8;i++) r._reversible[i]=true;
    r.throttle=.01f; r._roll_in=.08f; r.run();
    assert(!r.limit.roll && r._thrust_rpyt_out[2]<0);
    std::mt19937 gen(42); std::uniform_real_distribution<float> demand(-1,1), throttle(0,1);
    for(int n=0;n<20000;n++) {
        Mixer s; s.throttle=throttle(gen); s._roll_in=demand(gen);
        s._pitch_in=demand(gen); s._yaw_in=demand(gen); s.lateral=demand(gen);
        s._roll_offset=.785f*demand(gen); s._pitch_offset=.785f*demand(gen); s.run();
        for(int i=0;i<8;i++) {
            assert(std::isfinite(s._thrust_rpyt_out[i]));
            assert(s._thrust_rpyt_out[i]<=1 && s._thrust_rpyt_out[i]>=(i%2 ? -1 : 0));
        }
    }
    puts("PASS: production allocator lower/upper bounds, low-demand compatibility, lateral selection, reversible compatibility, production integral saturation/unwind, 20000 mixed commands with offsets up to 45 degrees");
}
'''
with tempfile.TemporaryDirectory(prefix='lift-limits-') as directory:
    cpp = Path(directory)/'check.cpp'
    binary = Path(directory)/'check'
    cpp.write_text(harness + matrix + init + function + pid + checks)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
