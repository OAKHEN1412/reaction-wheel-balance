"""
sim/model.py
============
โมเดลจำลอง "ตรงตามสมการที่เฟิร์มแวร์ใช้จริง" (contract กับทีม firmware/):

    a = Kp*theta + Kd*theta_dot + Kw*omega_w + Ki*integral(theta)      [rad/s^2]
    omega_cmd += a * dt_ctrl
    omega_cmd = clip(omega_cmd, -omega_max, omega_max)
    (deadband ใกล้ 0 ก่อนส่งให้มอเตอร์)

โครงสร้างไฟล์:
    - plant_deriv()   : สมการพลวัตของกลไก (ดูอนุพันธ์เต็มใน sim/params.py::derive และ sim/README.md)
    - Sensors()       : จำลอง IMU (complementary filter) + FG tach (quantize/noise) + gyro bias
    - Controller()    : ควบคุมแบบ PD+Kw+I ตาม contract, มี anti-windup + deadband + delay 1 sample
    - motor_step()    : ลูปความเร็วมอเตอร์ 1st-order + torque/สปีดลิมิต
    - simulate()      : ห่อรวมทั้งหมด, ฟิสิกส์ step เล็ก (0.5 ms, RK4), ควบคุม step 2 ms

สัญลักษณ์เครื่องหมาย (sign convention) — สำคัญมาก อ่านเพิ่มใน README:
    theta > 0  หมายถึงเฟรมเอียงไปทางที่ sin(theta) ทำให้ theta_dd > 0 มากขึ้น (ทิศทางล้ม)
    a > 0      หมายถึงคำสั่งเร่งล้อ "สัมพัทธ์กับเฟรม" ไปทาง + ซึ่งจากสมการจะสร้าง theta_dd
               ในทาง "ลบ" (หน่วงการล้มทาง +) เมื่อ Kp มีเครื่องหมายบวก ->  Kp ต้องเป็น "บวก"
               (ดู sim/design_gains.py ที่พิสูจน์ด้วย LQR + เทสต์ sign convention ใน test_model.py)
"""

from dataclasses import dataclass, field
import numpy as np


# ---------------------------------------------------------------------------
# Plant (กลไก)
# ---------------------------------------------------------------------------
def plant_deriv(theta: float, thetad: float, omega_w: float, omega_cmd: float,
                 d: dict, tau_ext: float = 0.0):
    """
    คืน (thetad, thetadd, domega_w) ตามสมการ Lagrangian ที่รวม tau_w ออกแล้ว:

        domega_w = clip((omega_cmd - omega_w)/tau_m, -alpha_max, alpha_max)
        theta_dd = (B*sin(theta) - I_w*domega_w + tau_ext) / (C + I_w)

    tau_ext: แรงบิดรบกวนจากภายนอกที่กระทำรอบจุดหมุน (ใช้ทดสอบ disturbance rejection)
    """
    domega_w = (omega_cmd - omega_w) / d["tau_m"]
    domega_w = np.clip(domega_w, -d["alpha_max"], d["alpha_max"])
    thetadd = (d["B"] * np.sin(theta) - d["I_w"] * domega_w + tau_ext) / (d["C"] + d["I_w"])
    return thetad, thetadd, domega_w


def rk4_step(theta, thetad, omega_w, omega_cmd, d, dt, tau_ext=0.0):
    def f(s):
        th, thd, ow = s
        dth, dthd, dow = plant_deriv(th, thd, ow, omega_cmd, d, tau_ext)
        return np.array([dth, dthd, dow])

    s0 = np.array([theta, thetad, omega_w])
    k1 = f(s0)
    k2 = f(s0 + 0.5 * dt * k1)
    k3 = f(s0 + 0.5 * dt * k2)
    k4 = f(s0 + dt * k3)
    s1 = s0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return s1[0], s1[1], s1[2]


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------
@dataclass
class Sensors:
    d: dict
    rng: np.random.Generator
    noisy: bool = True
    theta_hat: float = 0.0
    gyro_bias: float = 0.0
    accel_bias: float = 0.0    # rad, systematic (คงที่, ไม่ใช่สุ่ม) IMU/mounting offset error
    _initialized: bool = False

    def __post_init__(self):
        if self.noisy:
            self.gyro_bias = self.rng.normal(0.0, self.d["gyro_bias_std"])

    def sample(self, theta_true: float, thetad_true: float, omega_w_true: float):
        """คืน (theta_meas_complementary, thetad_meas(gyro), omega_w_meas)"""
        d = self.d
        if self.noisy:
            accel_angle = theta_true + self.accel_bias + self.rng.normal(0.0, d["accel_noise_std"])
            gyro_meas = thetad_true + self.gyro_bias + self.rng.normal(0.0, d["gyro_noise_std"])
            omega_meas = omega_w_true + self.rng.normal(0.0, d["omega_w_noise_std"])
            q = d["omega_w_quant"]
            if q > 0:
                omega_meas = np.round(omega_meas / q) * q
        else:
            accel_angle = theta_true + self.accel_bias
            gyro_meas = thetad_true
            omega_meas = omega_w_true

        if not self._initialized:
            # warm-start: ของจริงบนบอร์ดจะ calibrate/seed มุมเริ่มต้นจาก accel ก่อนเริ่มลูป
            # ไม่ปล่อยให้ theta_hat เริ่มจาก 0 แล้วค่อยไล่ตามทีละนิด (จะช้าเกินไปสำหรับระบบที่
            # ล้มเร็ว/time constant สั้นแบบนี้)
            self.theta_hat = accel_angle
            self._initialized = True

        alpha = d["comp_alpha"]
        self.theta_hat = alpha * (self.theta_hat + gyro_meas * d["dt_ctrl"]) + (1 - alpha) * accel_angle
        return self.theta_hat, gyro_meas, omega_meas


# ---------------------------------------------------------------------------
# Controller (ตรงตาม contract ของ firmware)
# ---------------------------------------------------------------------------
@dataclass
class Controller:
    Kp: float
    Kd: float
    Kw: float
    Ki: float
    omega_max: float
    dt_ctrl: float
    deadband: float = 0.3          # rad/s, บริเวณตายก่อนส่งให้มอเตอร์ (~stiction ของ ESC จริง,
                                    # ประมาณ ~3 rpm ที่ล้อ -- เล็กพอไม่ให้หน่วงการตอบสนองของ
                                    # ระบบที่ไม่เสถียรเร็ว (open-loop time-to-double ~0.1s)
    i_limit: float | None = None   # limit ของ |integral| (anti-windup clamp), None = auto
    delay_samples: int = 1

    integral: float = 0.0
    omega_cmd: float = 0.0
    _buf: list = field(default_factory=list)

    def __post_init__(self):
        if self.i_limit is None:
            # จำกัด "การหน่วง" ของ integral term ไม่ให้เกิน ~50% ของ omega_max ต่อ 1 คำสั่ง a
            self.i_limit = (0.5 * self.omega_max) / max(abs(self.Ki), 1e-9) if self.Ki != 0 else 1e9

    def reset(self):
        self.integral = 0.0
        self.omega_cmd = 0.0
        self._buf = []

    def step(self, theta_meas: float, thetad_meas: float, omega_w_meas: float):
        """
        คำนวณ 1 รอบควบคุม (2 ms). คืน (omega_cmd_to_motor, a, saturated: bool)
        รองรับ delay_samples หน่วงข้อมูลก่อนใช้จริง (1 sample delay ตามที่ระบุใน spec)
        """
        self._buf.append((theta_meas, thetad_meas, omega_w_meas))
        if len(self._buf) <= self.delay_samples:
            # ยังไม่มีข้อมูลเก่าพอ ใช้ค่าปัจจุบันไปก่อน (ช่วง warm-up สั้นๆ)
            theta_u, thetad_u, omega_u = theta_meas, thetad_meas, omega_w_meas
        else:
            theta_u, thetad_u, omega_u = self._buf.pop(0)

        a = self.Kp * theta_u + self.Kd * thetad_u + self.Kw * omega_u + self.Ki * self.integral

        omega_cmd_new = self.omega_cmd + a * self.dt_ctrl
        saturated = abs(omega_cmd_new) > self.omega_max
        omega_cmd_new = float(np.clip(omega_cmd_new, -self.omega_max, self.omega_max))
        self.omega_cmd = omega_cmd_new

        # anti-windup: อย่า integrate ต่อถ้า saturated และ error มีเครื่องหมายเดียวกับที่ทำให้ saturate มากขึ้น
        would_push_further = (self.omega_cmd >= self.omega_max and theta_u > 0) or \
                              (self.omega_cmd <= -self.omega_max and theta_u < 0)
        if not (saturated and would_push_further):
            self.integral += theta_u * self.dt_ctrl
            self.integral = float(np.clip(self.integral, -self.i_limit, self.i_limit))

        # deadband: คำสั่งเล็กเกินไปไม่ส่งให้มอเตอร์ (สติกชัน/deadzone ของ ESC จริง)
        omega_out = self.omega_cmd if abs(self.omega_cmd) > self.deadband else 0.0

        return omega_out, a, saturated


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------
def simulate(d: dict, gains: dict, theta0: float = 0.0, thetad0: float = 0.0,
             omega_w0: float = 0.0, T: float = 5.0, dt_phys: float = 0.0005,
             noisy: bool = True, seed: int = 0, disturbance=None,
             open_loop: bool = False, accel_bias: float = 0.0):
    """
    d       : derived params dict (จาก params.derive())
    gains   : {'Kp':..,'Kd':..,'Kw':..,'Ki':..}
    disturbance(t) -> tau_ext (N*m), None = ไม่มี
    open_loop: True = ปล่อย omega_cmd=0 เสมอ (ไม่มีคอนโทรลเลอร์) สำหรับเทส "ล้มเอง"
    accel_bias: rad, ความคลาดเคลื่อนคงที่ของ IMU/การติดตั้ง (ไม่ใช่ noise สุ่ม) -- ใช้ทดสอบว่า
                Kw/Ki ทำให้ wheel speed ยังคง bounded ได้ไหมเมื่อมี sensor offset ถาวร

    คืน dict of np.array: t, theta, thetad, omega_w, omega_cmd, a, theta_hat, saturated
    """
    dt_ctrl = d["dt_ctrl"]
    n_phys_per_ctrl = max(1, round(dt_ctrl / dt_phys))
    dt_phys = dt_ctrl / n_phys_per_ctrl  # ปรับให้หารกันพอดี

    n_ctrl_steps = int(round(T / dt_ctrl))

    rng = np.random.default_rng(seed)
    sensors = Sensors(d=d, rng=rng, noisy=noisy, accel_bias=accel_bias)
    ctrl = Controller(Kp=gains["Kp"], Kd=gains["Kd"], Kw=gains["Kw"], Ki=gains["Ki"],
                       omega_max=d["omega_max"], dt_ctrl=dt_ctrl)

    theta, thetad, omega_w = theta0, thetad0, omega_w0
    t = 0.0

    hist = {k: [] for k in ["t", "theta", "thetad", "omega_w", "omega_cmd", "a",
                             "theta_hat", "saturated"]}

    omega_cmd_current = 0.0
    for i in range(n_ctrl_steps):
        theta_hat, thetad_meas, omega_meas = sensors.sample(theta, thetad, omega_w)

        if open_loop:
            omega_cmd_current, a_cmd, sat = 0.0, 0.0, False
        else:
            omega_cmd_current, a_cmd, sat = ctrl.step(theta_hat, thetad_meas, omega_meas)

        tau_ext = disturbance(t) if disturbance is not None else 0.0

        for _ in range(n_phys_per_ctrl):
            theta, thetad, omega_w = rk4_step(theta, thetad, omega_w, omega_cmd_current,
                                               d, dt_phys, tau_ext)
            t += dt_phys

        hist["t"].append(t)
        hist["theta"].append(theta)
        hist["thetad"].append(thetad)
        hist["omega_w"].append(omega_w)
        hist["omega_cmd"].append(omega_cmd_current)
        hist["a"].append(a_cmd)
        hist["theta_hat"].append(theta_hat)
        hist["saturated"].append(sat)

    return {k: np.array(v) for k, v in hist.items()}
