"""
sim/test_model.py
==================
เทสต์พื้นฐาน (TDD mindset) สำหรับ sim/model.py + sim/params.py

รัน:  py -m unittest sim.test_model -v      (จาก root โปรเจค)
หรือ  py -m unittest test_model -v          (จากในโฟลเดอร์ sim/)
"""

import sys
import os
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import params
import model
import design_gains as dg

# ใช้เกนจาก design_gains.py ตรงๆ (ไม่ hardcode) กันไม่ให้เทสต์กับค่าที่ล้าสมัยเมื่อ DEFAULT_Q
# หรือ params.py เปลี่ยน
NOMINAL_GAINS = dg.lqr_gains_dict(params.nominal_derived(), **dg.DEFAULT_Q)


class TestOpenLoopInstability(unittest.TestCase):
    """upright แบบ open-loop (ไม่มีคอนโทรลเลอร์) ต้องไม่เสถียร -- ล้มเอง"""

    def test_small_tilt_grows_without_control(self):
        d = params.nominal_derived()
        h = model.simulate(d, dict(Kp=0, Kd=0, Kw=0, Ki=0), theta0=np.deg2rad(2.0),
                            T=1.0, noisy=False, open_loop=True)
        self.assertGreater(abs(h["theta"][-1]), abs(h["theta"][0]) * 3,
                            "มุมเอียงต้องขยายตัวขึ้นแบบ open-loop (unstable)")
        # ทิศทางการล้มต้องสอดคล้องกับสัญญาณเริ่มต้น (theta0>0 -> ล้มไปทาง + ก่อน จนกว่าจะพลิก)
        self.assertGreater(h["theta"][10], 0)

    def test_zero_tilt_stays_put_open_loop(self):
        """จุดสมดุล theta=0 เป๊ะ (ไม่มี disturbance) ต้องนิ่ง แม้ไม่มีคอนโทรลเลอร์
        (สมดุลไม่เสถียรแต่ยังเป็นจุดสมดุลทางคณิตศาสตร์)"""
        d = params.nominal_derived()
        h = model.simulate(d, dict(Kp=0, Kd=0, Kw=0, Ki=0), theta0=0.0, thetad0=0.0,
                            T=0.5, noisy=False, open_loop=True)
        self.assertLess(np.max(np.abs(h["theta"])), 1e-6)


class TestMomentumSanity(unittest.TestCase):
    """แรงบิดภายใน (การเร่งล้อ) ต้องไม่สร้าง/ทำลายโมเมนตัมเชิงมุมรวมของระบบ เมื่อไม่มี
    แรงโน้มถ่วง/แรงภายนอก (sanity check ของสมการ Lagrangian ที่ผูก tau_w ระหว่างเฟรม-ล้อ)"""

    def test_wheel_spin_up_with_no_gravity_conserves_total_momentum(self):
        d = dict(params.nominal_derived())
        d["B"] = 0.0  # ปิดแรงโน้มถ่วง (ทดสอบเฉพาะคู่แรงภายใน เฟรม<->ล้อ)

        theta, thetad, omega_w = 0.0, 0.0, 0.0
        omega_cmd = 100.0  # สั่งให้ล้อเร่งไปทาง + อย่างต่อเนื่อง (จำลองด้วยมือ ไม่ผ่าน controller)
        L0 = (d["C"] + d["I_w"]) * thetad + d["I_w"] * omega_w

        dt = 0.0005
        for _ in range(2000):  # 1 วินาที
            theta, thetad, omega_w = model.rk4_step(theta, thetad, omega_w, omega_cmd, d, dt)

        L1 = (d["C"] + d["I_w"]) * thetad + d["I_w"] * omega_w
        self.assertAlmostEqual(L0, L1, places=6,
                                msg="โมเมนตัมเชิงมุมรวมของระบบ (เฟรม+ล้อ) ต้องคงที่เมื่อไม่มี"
                                    "แรงบิดภายนอก (แรงมอเตอร์เป็นแรงภายในคู่ที่หักล้างกัน)")
        # เฟรมต้องหมุนไปทาง "ตรงข้าม" กับล้อ (reaction) เมื่อไม่มีแรงโน้มถ่วงคอยดึง
        self.assertLess(thetad, 0.0, "เมื่อล้อเร่งไปทาง +, เฟรม (ไม่มีแรงโน้มถ่วง) ต้องหมุนไปทาง -"
                                      " (แรงปฏิกิริยา, Newton's 3rd law)")


class TestClosedLoopRecovery(unittest.TestCase):
    """ลูปปิดด้วยเกน nominal ต้องกู้คืนจากมุมเอียง 5 องศา (ขอบเขตโครงงาน) ได้"""

    def test_recovers_from_5deg_noiseless(self):
        d = params.nominal_derived()
        h = model.simulate(d, NOMINAL_GAINS, theta0=np.deg2rad(5.0), T=4.0, noisy=False, seed=0)
        tail = h["theta"][-500:]
        self.assertLess(np.max(np.abs(tail)), np.deg2rad(1.0),
                         "theta ต้องเข้าใกล้ 0 (settled) ภายใน 4 วินาที")
        self.assertLess(np.max(np.abs(h["theta"])), np.deg2rad(60),
                         "ต้องไม่ diverge ระหว่างทาง")

    def test_recovers_from_5deg_with_noise_multiple_seeds(self):
        d = params.nominal_derived()
        n_ok = 0
        n_trials = 8
        for seed in range(n_trials):
            h = model.simulate(d, NOMINAL_GAINS, theta0=np.deg2rad(5.0), T=4.0,
                                noisy=True, seed=seed)
            tail = h["theta"][-500:]
            if np.max(np.abs(tail)) < np.deg2rad(2.0) and np.max(np.abs(h["theta"])) < np.deg2rad(60):
                n_ok += 1
        self.assertGreaterEqual(n_ok, n_trials - 1,
                                 f"ต้อง settle ได้อย่างน้อย {n_trials-1}/{n_trials} รอบ (มี noise)")


class TestSignConvention(unittest.TestCase):
    """ตรวจ sign convention: theta>0 -> a (คำสั่งของคอนโทรลเลอร์) ต้องมีเครื่องหมายที่ทำให้เกิด
    แรงบิดหักล้าง (restoring), และ Kp ต้องเป็นบวกตาม convention ที่ระบุใน model.py/README"""

    def test_positive_theta_gives_restoring_wheel_command(self):
        ctrl = model.Controller(Kp=NOMINAL_GAINS["Kp"], Kd=0.0, Kw=0.0, Ki=0.0,
                                 omega_max=200.0, dt_ctrl=0.002, deadband=0.0, delay_samples=0)
        omega_out, a, sat = ctrl.step(theta_meas=np.deg2rad(5.0), thetad_meas=0.0, omega_w_meas=0.0)
        self.assertGreater(a, 0.0, "theta>0 กับ Kp>0 ต้องให้ a>0 (สั่งเร่งล้อไปทาง +)")

    def test_positive_wheel_accel_reduces_positive_theta_dd(self):
        """ยืนยันที่ระดับพลานต์ (ไม่ผ่านคอนโทรลเลอร์): omega_cmd>0 (เร่งล้อ +) ที่ theta>0 ต้อง
        ทำให้ theta_dd มีค่าน้อยกว่ากรณี omega_cmd=0 (คือมันหน่วง/ต้านการล้ม)"""
        d = params.nominal_derived()
        theta = np.deg2rad(5.0)
        _, thetadd_free, _ = model.plant_deriv(theta, 0.0, 0.0, omega_cmd=0.0, d=d)
        _, thetadd_actuated, _ = model.plant_deriv(theta, 0.0, 0.0, omega_cmd=d["omega_max"], d=d)
        self.assertLess(thetadd_actuated, thetadd_free,
                         "การเร่งล้อไปทาง + ต้องลด theta_dd ลง (ต้านการล้มไปทาง +)")

    def test_kp_sign_is_positive_in_nominal_gains(self):
        self.assertGreater(NOMINAL_GAINS["Kp"], 0.0)


class TestVoltageMode(unittest.TestCase):
    def controller(self, **kw):
        args = dict(Kp=100, Kd=0, Kw=0, Ki=0, omega_max=60,
                    dt_ctrl=0.002, deadband=0, delay_samples=0)
        args.update(kw)
        return model.Controller(**args)

    def test_inverse_model_and_signed_back_emf(self):
        d = params.nominal_derived()
        for omega in (-12, 12):
            c = self.controller()
            target, a, sat = c.step(0.1, 0, omega)
            self.assertAlmostEqual(target, omega + 0.18 * a)
            self.assertAlmostEqual(model.plant_deriv(0, 0, omega, target, d)[2], a)
            self.assertFalse(sat)

    def test_voltage_clipping_and_antiwindup(self):
        c = self.controller(Ki=1)
        for sign in (1, -1):
            target, _, sat = c.step(sign * 10, 0, 0)
            self.assertEqual(target, sign * 60)
            self.assertTrue(sat)
            self.assertEqual(c.integral, 0)

    def test_speed_mode_remains_integrated(self):
        c = self.controller(control_mode="speed")
        self.assertAlmostEqual(c.step(0.1, 0, 12)[0], 0.02)
        self.assertAlmostEqual(c.step(0.1, 0, 12)[0], 0.04)

    def test_current_limit_and_duty_limit(self):
        d = params.nominal_derived()
        self.assertEqual(model.plant_deriv(0, 0, 0, 1e6, d)[2], d["alpha_max"])
        d["alpha_max"] = 1e6
        self.assertAlmostEqual(model.plant_deriv(0, 0, 0, 1e6, d)[2],
                               d["omega_max"] / d["tau_m"])

    def test_coast_asymmetry_both_directions(self):
        d = params.nominal_derived()
        weak = dict(d, coast_decel_frac=0.3)
        for sign in (-1, 1):
            for target in (0, 10):
                normal = model.plant_deriv(0, 0, sign * 20, sign * target, d)[2]
                self.assertAlmostEqual(model.plant_deriv(0, 0, sign * 20, sign * target, weak)[2], 0.3 * normal)
            for target in (-10, 30):
                self.assertEqual(model.plant_deriv(0, 0, sign * 20, sign * target, weak)[2],
                                 model.plant_deriv(0, 0, sign * 20, sign * target, d)[2])

    def test_reversal_inserts_zero_duty_interval(self):
        d = params.nominal_derived()
        commands = [(20.0, 0, False), (-20.0, 0, False), (-20.0, 0, False)]
        with patch.object(model.Controller, "step", side_effect=commands):
            h = model.simulate(d, NOMINAL_GAINS, T=3*d["dt_ctrl"], noisy=False)
        np.testing.assert_array_equal(h["omega_cmd"], [20.0, 0.0, -20.0])
        self.assertGreater(h["omega_w"][1], 0)  # still spinning during blank


    def test_speed_guard_allows_reverse_torque(self):
        c = self.controller()
        self.assertEqual(c.step(1, 0, 55)[0], 55)
        self.assertLess(c.step(-1, 0, 55)[0], 55)


if __name__ == "__main__":
    unittest.main(verbosity=2)
