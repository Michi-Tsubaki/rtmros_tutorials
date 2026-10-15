import unittest

import numpy as np

from nextage_tutorials.ft_payload import compensate, fit_payload, gravity_from_quaternion
from nextage_tutorials.ft_zero import estimate_bias


class PayloadTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(42)
        self.g = rng.normal(size=(20, 3))
        self.g *= 9.80665 / np.linalg.norm(self.g, axis=1)[:, None]
        self.mass = 0.24
        self.cog = np.array([0.02, -0.03, 0.07])
        self.bias = np.array([0.4, -0.2, 0.8, 0.01, -0.02, 0.03])
        f = self.mass * self.g
        self.w = np.c_[f, np.cross(self.cog, f)] + self.bias

    def test_recovers_payload_and_six_biases(self):
        result = fit_payload(self.g, self.w)
        self.assertAlmostEqual(result['mass'], self.mass)
        np.testing.assert_allclose(result['center_of_mass'], self.cog, atol=1e-10)
        np.testing.assert_allclose(result['force_bias']+result['torque_bias'], self.bias, atol=1e-10)
        for g, w in zip(self.g, self.w):
            np.testing.assert_allclose(compensate(w, g, result), 0, atol=1e-10)

    def test_one_orientation_is_not_identifiable(self):
        with self.assertRaisesRegex(ValueError, 'orientation diversity'):
            fit_payload(np.tile(self.g[0], (6, 1)), np.tile(self.w[0], (6, 1)))

    def test_contact_contamination_is_rejected(self):
        self.w[0, 0] += 20.0
        with self.assertRaisesRegex(ValueError, 'residual'):
            fit_payload(self.g, self.w)

    def test_inverted_sensor_polarity_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'mass'):
            fit_payload(self.g, -self.w)

    def test_static_noise_gate(self):
        samples = np.tile(self.bias, (100, 1))
        bias, _ = estimate_bias(samples, 0.2, 0.02)
        np.testing.assert_allclose(bias, self.bias)
        samples[0, 5] += 1.0
        with self.assertRaisesRegex(ValueError, 'not stable'):
            estimate_bias(samples, 0.2, 0.02)

    def test_gravity_transform(self):
        np.testing.assert_allclose(gravity_from_quaternion([0, 0, 0, 1]), [0, 0, -9.80665])
        np.testing.assert_allclose(gravity_from_quaternion([0, np.sqrt(0.5), 0, np.sqrt(0.5)]),
                                   [-9.80665, 0, 0], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
